#include "sim.h"

#include <math.h>

#define PLANT_GAIN_MM_S2 2000.0f
#define PLANT_DAMPING_1_S 2.1f
#define PLANT_FRICTION_MM_S2 30.0f
#define PLANT_STROKE_MM 520.0f

static const sim_scenario_t kScenarios[] = {
    {"homing_and_move", 120.0f, 300.0f,   0.0f, 1.0e9f,   0.0f, false, 15.0f},
    {"two_moves",       120.0f, 300.0f, 100.0f,    9.0f,   0.0f, false, 16.0f},
    {"obstacle_stall",  120.0f, 300.0f,   0.0f, 1.0e9f, 200.0f, false, 15.0f},
    {"switch_stuck",    120.0f, 300.0f,   0.0f, 1.0e9f,   0.0f, true,  12.0f},
};

const sim_scenario_t *sim_scenarios(size_t *count)
{
    *count = sizeof(kScenarios) / sizeof(kScenarios[0]);
    return kScenarios;
}

const char *sim_mode_name(slide_mode_t mode)
{
    switch (mode) {
    case SLIDE_DISABLED: return "DISABLED";
    case SLIDE_HOMING:   return "HOMING";
    case SLIDE_POSITION: return "POSITION";
    case SLIDE_FAULT:    return "FAULT";
    default:             return "?";
    }
}

sim_result_t sim_run(const sim_scenario_t *sc, const slide_config_t *cfg, FILE *csv)
{
    slide_state_t state;
    sim_result_t r;
    float pos = sc->start_pos_mm;
    float vel = 0.0f;
    float upper = PLANT_STROKE_MM;
    size_t steps = (size_t)(sc->duration_s / cfg->dt_s + 0.5f);
    size_t i;

    if (sc->obstacle_pos_mm > 0.0f) {
        upper = sc->obstacle_pos_mm;
    }

    slide_init(&state);
    r.homed = false;
    r.final_mode = SLIDE_DISABLED;
    r.time_to_fault_s = -1.0f;
    r.final_pos_mm = pos;
    r.max_pos_mm = pos;
    r.max_speed_mm_s = 0.0f;
    r.max_abs_duty = 0.0f;

    if (csv != NULL) {
        fprintf(csv, "t,pos,vel,duty,mode,target\n");
    }

    for (i = 0; i < steps; i++) {
        float t = (float)i * cfg->dt_s;
        float target = (t >= sc->target2_time_s) ? sc->target2_mm : sc->target1_mm;
        slide_input_t in;
        slide_output_t out;
        float acc;

        in.enabled = true;
        in.encoder_ticks = (int32_t)floorf((pos - sc->start_pos_mm) / cfg->mm_per_tick);
        in.bottom_switch = !sc->switch_stuck && pos <= SIM_SWITCH_POS_MM;
        in.target_mm = target;

        out = slide_step(&state, cfg, &in);

        acc = PLANT_GAIN_MM_S2 * out.duty - PLANT_DAMPING_1_S * vel;
        if (vel > 0.0f) {
            acc -= PLANT_FRICTION_MM_S2;
        } else if (vel < 0.0f) {
            acc += PLANT_FRICTION_MM_S2;
        }
        vel += acc * cfg->dt_s;
        pos += vel * cfg->dt_s;
        if (pos <= 0.0f) {
            pos = 0.0f;
            vel = (vel > 0.0f) ? vel : 0.0f;
        }
        if (pos >= upper) {
            pos = upper;
            vel = (vel < 0.0f) ? vel : 0.0f;
        }

        r.homed = out.homed;
        r.final_mode = out.mode;
        if (out.mode == SLIDE_FAULT && r.time_to_fault_s < 0.0f) {
            r.time_to_fault_s = t;
        }
        if (pos > r.max_pos_mm) {
            r.max_pos_mm = pos;
        }
        if (fabsf(vel) > r.max_speed_mm_s) {
            r.max_speed_mm_s = fabsf(vel);
        }
        if (fabsf(out.duty) > r.max_abs_duty) {
            r.max_abs_duty = fabsf(out.duty);
        }

        if (csv != NULL) {
            fprintf(csv, "%.2f,%.3f,%.2f,%.3f,%s,%.1f\n", (double)t, (double)pos,
                    (double)vel, (double)out.duty, sim_mode_name(out.mode),
                    (double)target);
        }
    }

    r.final_pos_mm = pos;
    return r;
}
