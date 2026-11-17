#ifndef SIM_H
#define SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "slide.h"

#define SIM_SWITCH_POS_MM 1.0f

typedef struct {
    const char *name;
    float start_pos_mm;
    float target1_mm;
    float target2_mm;
    float target2_time_s;
    float obstacle_pos_mm;
    bool  switch_stuck;
    float duration_s;
} sim_scenario_t;

typedef struct {
    bool         homed;
    slide_mode_t final_mode;
    float        time_to_fault_s;
    float        final_pos_mm;
    float        max_pos_mm;
    float        max_speed_mm_s;
    float        max_abs_duty;
} sim_result_t;

const sim_scenario_t *sim_scenarios(size_t *count);
sim_result_t sim_run(const sim_scenario_t *scenario, const slide_config_t *cfg,
                     FILE *csv);
const char *sim_mode_name(slide_mode_t mode);

#endif
