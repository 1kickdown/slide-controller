#ifndef SLIDE_H
#define SLIDE_H

#include <stdbool.h>
#include <stdint.h>

/* units: mm, mm/s, mm/s^2, s; duty in [-1, 1] */

typedef enum {
    SLIDE_DISABLED = 0,
    SLIDE_HOMING,
    SLIDE_POSITION,
    SLIDE_FAULT
} slide_mode_t;

typedef struct {
    float mm_per_tick;
    float pos_min_mm;
    float pos_max_mm;
    float homing_duty;
    float homing_timeout_s;
    float max_vel_mm_s;
    float max_acc_mm_s2;
    float kp;
    float ki;
    float kd;
    float kv;
    float ka;
    float integrator_max;
    float vel_filter_alpha;
    float stall_duty;
    float stall_vel_mm_s;
    float stall_time_s;
    float dt_s;
} slide_config_t;

typedef struct {
    bool    enabled;
    int32_t encoder_ticks;
    bool    bottom_switch;
    float   target_mm;
} slide_input_t;

typedef struct {
    slide_mode_t mode;
    float        duty;
    bool         homed;
} slide_output_t;

typedef struct {
    slide_mode_t mode;
    bool         homed;
    int32_t      tick_offset;
    float        home_timer_s;
    float        sp_pos_mm;
    float        sp_vel_mm_s;
    float        vel_est_mm_s;
    float        prev_pos_mm;
    float        integrator;
    float        stall_timer_s;
} slide_state_t;

void           slide_default_config(slide_config_t *cfg);
void           slide_init(slide_state_t *state);
slide_output_t slide_step(slide_state_t *state, const slide_config_t *cfg,
                          const slide_input_t *in);

#endif
