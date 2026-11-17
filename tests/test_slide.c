#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sim.h"
#include "slide.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        g_checks++;                                                             \
        if (!(cond)) {                                                          \
            g_failures++;                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
        }                                                                       \
    } while (0)

static slide_input_t make_input(bool enabled, int32_t ticks, bool sw, float target)
{
    slide_input_t in;

    in.enabled = enabled;
    in.encoder_ticks = ticks;
    in.bottom_switch = sw;
    in.target_mm = target;
    return in;
}

static void home(slide_state_t *st, const slide_config_t *cfg)
{
    slide_input_t in = make_input(true, 0, false, 0.0f);

    slide_init(st);
    (void)slide_step(st, cfg, &in);
    in.bottom_switch = true;
    (void)slide_step(st, cfg, &in);
}

static const sim_scenario_t *find_scenario(const char *name)
{
    size_t n;
    size_t i;
    const sim_scenario_t *sc = sim_scenarios(&n);

    for (i = 0; i < n; i++) {
        if (strcmp(sc[i].name, name) == 0) {
            return &sc[i];
        }
    }
    return NULL;
}

static void test_disabled_outputs_zero(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(false, 100, false, 50.0f);
    slide_output_t out;

    slide_default_config(&cfg);
    slide_init(&st);
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_DISABLED);
    CHECK(out.duty == 0.0f);
    CHECK(!out.homed);
}

static void test_homing_drives_towards_switch(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 500, false, 100.0f);
    slide_output_t out;

    slide_default_config(&cfg);
    slide_init(&st);
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_HOMING);
    CHECK(out.duty == cfg.homing_duty);
    CHECK(!out.homed);
}

static void test_switch_zeroes_position(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 1234, false, 0.0f);
    slide_output_t out;

    slide_default_config(&cfg);
    slide_init(&st);
    (void)slide_step(&st, &cfg, &in);
    in.bottom_switch = true;
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_POSITION);
    CHECK(out.homed);
    CHECK(st.tick_offset == 1234);
    /* At the home position with target 0 the controller should not push. */
    in.bottom_switch = false;
    out = slide_step(&st, &cfg, &in);
    CHECK(fabsf(out.duty) < 0.05f);
}

static void test_homing_timeout_faults(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 100.0f);
    slide_output_t out = {SLIDE_DISABLED, 0.0f, false};
    int i;
    int steps;

    slide_default_config(&cfg);
    slide_init(&st);
    steps = (int)(cfg.homing_timeout_s / cfg.dt_s) + 5;
    for (i = 0; i < steps; i++) {
        out = slide_step(&st, &cfg, &in);
    }
    CHECK(out.mode == SLIDE_FAULT);
    CHECK(out.duty == 0.0f);
}

static void test_invalid_input_faults(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in;
    slide_output_t out;

    slide_default_config(&cfg);
    home(&st, &cfg);
    in = make_input(true, 0, false, NAN);
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_FAULT);
    CHECK(out.duty == 0.0f);
}

static void test_invalid_config_faults(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 10.0f);
    slide_output_t out;

    slide_default_config(&cfg);
    cfg.dt_s = 0.0f;
    slide_init(&st);
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_FAULT);
}

static void test_fault_latched_until_disabled(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in;
    slide_output_t out;

    slide_default_config(&cfg);
    home(&st, &cfg);
    in = make_input(true, 0, false, NAN);
    (void)slide_step(&st, &cfg, &in);

    in = make_input(true, 0, false, 100.0f); /* valid again, still latched */
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_FAULT);
    CHECK(out.duty == 0.0f);

    in.enabled = false;
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_DISABLED);
    CHECK(!out.homed);

    in.enabled = true; /* a new run starts with homing */
    out = slide_step(&st, &cfg, &in);
    CHECK(out.mode == SLIDE_HOMING);
}

static void test_target_clamped_and_duty_bounded(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 9999.0f);
    int i;

    slide_default_config(&cfg);
    cfg.stall_time_s = 1.0e6f; /* keep the encoder frozen without a stall fault */
    home(&st, &cfg);
    for (i = 0; i < 3000; i++) {
        slide_output_t out = slide_step(&st, &cfg, &in);

        CHECK(st.sp_pos_mm <= cfg.pos_max_mm + 1e-3f);
        CHECK(fabsf(out.duty) <= 1.0f);
    }
    CHECK(fabsf(st.sp_pos_mm - cfg.pos_max_mm) < 1e-3f);
}

static void test_profile_respects_limits(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 400.0f);
    float prev_vel = 0.0f;
    int i;

    slide_default_config(&cfg);
    cfg.stall_time_s = 1.0e6f;
    home(&st, &cfg);
    for (i = 0; i < 1500; i++) {
        (void)slide_step(&st, &cfg, &in);
        CHECK(fabsf(st.sp_vel_mm_s) <= cfg.max_vel_mm_s + 1e-3f);
        CHECK(fabsf(st.sp_vel_mm_s - prev_vel) / cfg.dt_s <= cfg.max_acc_mm_s2 + 1e-2f);
        prev_vel = st.sp_vel_mm_s;
    }
}

static void test_integrator_is_limited(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 400.0f);
    int i;

    slide_default_config(&cfg);
    cfg.stall_time_s = 1.0e6f;
    home(&st, &cfg);
    for (i = 0; i < 5000; i++) {
        (void)slide_step(&st, &cfg, &in);
    }
    CHECK(fabsf(st.integrator) <= cfg.integrator_max + 1e-6f);
}

static void test_stall_detection(void)
{
    slide_config_t cfg;
    slide_state_t st;
    slide_input_t in = make_input(true, 0, false, 400.0f);
    slide_output_t out = {SLIDE_DISABLED, 0.0f, false};
    int i;

    slide_default_config(&cfg);
    home(&st, &cfg);
    for (i = 0; i < 500; i++) { /* encoder never moves: blocked axis */
        out = slide_step(&st, &cfg, &in);
        if (out.mode == SLIDE_FAULT) {
            break;
        }
    }
    CHECK(out.mode == SLIDE_FAULT);
    CHECK(out.duty == 0.0f);
}

static void test_scenario_homing_and_move(void)
{
    slide_config_t cfg;
    sim_result_t r;

    slide_default_config(&cfg);
    r = sim_run(find_scenario("homing_and_move"), &cfg, NULL);
    CHECK(r.homed);
    CHECK(r.final_mode == SLIDE_POSITION);
    CHECK(fabsf(r.final_pos_mm - 300.0f) < 1.5f);
    CHECK(r.max_pos_mm - 300.0f < 5.0f);                 /* overshoot  */
    CHECK(r.max_speed_mm_s < 1.1f * cfg.max_vel_mm_s);   /* speed limit */
}

static void test_scenario_two_moves(void)
{
    slide_config_t cfg;
    sim_result_t r;

    slide_default_config(&cfg);
    r = sim_run(find_scenario("two_moves"), &cfg, NULL);
    CHECK(r.final_mode == SLIDE_POSITION);
    CHECK(fabsf(r.final_pos_mm - 100.0f) < 1.5f);
}

static void test_scenario_obstacle_stall(void)
{
    slide_config_t cfg;
    sim_result_t r;

    slide_default_config(&cfg);
    r = sim_run(find_scenario("obstacle_stall"), &cfg, NULL);
    CHECK(r.final_mode == SLIDE_FAULT);
    CHECK(r.time_to_fault_s > 3.0f);
    CHECK(r.time_to_fault_s < 10.0f);
    CHECK(r.max_pos_mm <= 200.0f + 1e-3f);
}

static void test_scenario_switch_stuck(void)
{
    slide_config_t cfg;
    sim_result_t r;

    slide_default_config(&cfg);
    r = sim_run(find_scenario("switch_stuck"), &cfg, NULL);
    CHECK(!r.homed);
    CHECK(r.final_mode == SLIDE_FAULT);
    CHECK(fabsf(r.time_to_fault_s - cfg.homing_timeout_s) < 0.1f);
}

int main(void)
{
    test_disabled_outputs_zero();
    test_homing_drives_towards_switch();
    test_switch_zeroes_position();
    test_homing_timeout_faults();
    test_invalid_input_faults();
    test_invalid_config_faults();
    test_fault_latched_until_disabled();
    test_target_clamped_and_duty_bounded();
    test_profile_respects_limits();
    test_integrator_is_limited();
    test_stall_detection();
    test_scenario_homing_and_move();
    test_scenario_two_moves();
    test_scenario_obstacle_stall();
    test_scenario_switch_stuck();

    printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
