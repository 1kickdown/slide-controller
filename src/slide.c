#include "slide.h"

#include <math.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static bool config_valid(const slide_config_t *c)
{
    return c->dt_s > 0.0f && c->mm_per_tick > 0.0f && c->pos_max_mm > c->pos_min_mm &&
           c->max_vel_mm_s > 0.0f && c->max_acc_mm_s2 > 0.0f &&
           c->homing_timeout_s > 0.0f && c->stall_time_s > 0.0f &&
           c->integrator_max >= 0.0f && c->vel_filter_alpha > 0.0f &&
           c->vel_filter_alpha <= 1.0f;
}

static slide_output_t make_output(const slide_state_t *s, float duty)
{
    slide_output_t out;

    out.mode = s->mode;
    out.duty = duty;
    out.homed = s->homed;
    return out;
}

void slide_default_config(slide_config_t *cfg)
{
    cfg->mm_per_tick = 0.05f;
    cfg->pos_min_mm = 0.0f;
    cfg->pos_max_mm = 500.0f;
    cfg->homing_duty = -0.08f;
    cfg->homing_timeout_s = 6.0f;
    cfg->max_vel_mm_s = 300.0f;
    cfg->max_acc_mm_s2 = 800.0f;
    cfg->kp = 0.03f;
    cfg->ki = 0.05f;
    cfg->kd = 0.0067f;
    cfg->kv = 0.00105f;
    cfg->ka = 0.0005f;
    cfg->integrator_max = 0.2f;
    cfg->vel_filter_alpha = 0.3f;
    cfg->stall_duty = 0.5f;
    cfg->stall_vel_mm_s = 5.0f;
    cfg->stall_time_s = 0.5f;
    cfg->dt_s = 0.02f;
}

void slide_init(slide_state_t *state)
{
    state->mode = SLIDE_DISABLED;
    state->homed = false;
    state->tick_offset = 0;
    state->home_timer_s = 0.0f;
    state->sp_pos_mm = 0.0f;
    state->sp_vel_mm_s = 0.0f;
    state->vel_est_mm_s = 0.0f;
    state->prev_pos_mm = 0.0f;
    state->integrator = 0.0f;
    state->stall_timer_s = 0.0f;
}

slide_output_t slide_step(slide_state_t *s, const slide_config_t *c,
                          const slide_input_t *in)
{
    float pos;
    float target;
    float err;
    float verr;
    float d;
    float v_lim;
    float v_des;
    float dv;
    float raw_vel;
    float duty_unsat;
    float duty;
    bool  sat_high;
    bool  sat_low;

    if (!in->enabled) {
        slide_init(s);
        return make_output(s, 0.0f);
    }
    if (s->mode == SLIDE_FAULT) {
        return make_output(s, 0.0f);
    }
    if (!config_valid(c) || !isfinite(in->target_mm)) {
        s->mode = SLIDE_FAULT;
        return make_output(s, 0.0f);
    }
    if (s->mode == SLIDE_DISABLED) {
        s->mode = SLIDE_HOMING;
        s->home_timer_s = 0.0f;
    }

    if (s->mode == SLIDE_HOMING) {
        if (in->bottom_switch) {
            s->tick_offset = in->encoder_ticks;
            s->homed = true;
            s->mode = SLIDE_POSITION;
            s->sp_pos_mm = 0.0f;
            s->sp_vel_mm_s = 0.0f;
            s->vel_est_mm_s = 0.0f;
            s->prev_pos_mm = 0.0f;
            s->integrator = 0.0f;
            s->stall_timer_s = 0.0f;
            return make_output(s, 0.0f);
        }
        s->home_timer_s += c->dt_s;
        if (s->home_timer_s > c->homing_timeout_s) {
            s->mode = SLIDE_FAULT;
            return make_output(s, 0.0f);
        }
        return make_output(s, c->homing_duty);
    }

    pos = (float)(in->encoder_ticks - s->tick_offset) * c->mm_per_tick;
    raw_vel = (pos - s->prev_pos_mm) / c->dt_s;
    s->vel_est_mm_s += c->vel_filter_alpha * (raw_vel - s->vel_est_mm_s);
    s->prev_pos_mm = pos;

    target = clampf(in->target_mm, c->pos_min_mm, c->pos_max_mm);
    d = target - s->sp_pos_mm;
    v_lim = fminf(c->max_vel_mm_s, sqrtf(2.0f * c->max_acc_mm_s2 * fabsf(d)));
    v_des = (d >= 0.0f) ? v_lim : -v_lim;
    dv = clampf(v_des - s->sp_vel_mm_s, -c->max_acc_mm_s2 * c->dt_s,
                c->max_acc_mm_s2 * c->dt_s);
    s->sp_vel_mm_s += dv;
    s->sp_pos_mm += s->sp_vel_mm_s * c->dt_s;
    if ((d >= 0.0f && s->sp_pos_mm > target) || (d < 0.0f && s->sp_pos_mm < target)) {
        s->sp_pos_mm = target;
    }

    err = s->sp_pos_mm - pos;
    verr = s->sp_vel_mm_s - s->vel_est_mm_s;
    duty_unsat = c->kp * err + c->kd * verr + c->kv * s->sp_vel_mm_s +
                 c->ka * (dv / c->dt_s) + s->integrator;
    duty = clampf(duty_unsat, -1.0f, 1.0f);

    sat_high = duty_unsat > 1.0f;
    sat_low = duty_unsat < -1.0f;
    if ((!sat_high && !sat_low) || (sat_high && err < 0.0f) || (sat_low && err > 0.0f)) {
        s->integrator += c->ki * err * c->dt_s;
        s->integrator = clampf(s->integrator, -c->integrator_max, c->integrator_max);
    }

    if (fabsf(duty) > c->stall_duty && fabsf(s->vel_est_mm_s) < c->stall_vel_mm_s) {
        s->stall_timer_s += c->dt_s;
    } else {
        s->stall_timer_s = 0.0f;
    }
    if (s->stall_timer_s > c->stall_time_s) {
        s->mode = SLIDE_FAULT;
        return make_output(s, 0.0f);
    }

    return make_output(s, duty);
}
