#include <stdio.h>
#include <string.h>

#include "sim.h"

int main(int argc, char **argv)
{
    slide_config_t cfg;
    size_t count;
    const sim_scenario_t *sc = sim_scenarios(&count);
    size_t i;

    slide_default_config(&cfg);

    if (argc > 1) {
        for (i = 0; i < count; i++) {
            if (strcmp(argv[1], sc[i].name) == 0) {
                (void)sim_run(&sc[i], &cfg, stdout);
                return 0;
            }
        }
        fprintf(stderr, "unknown scenario: %s\n", argv[1]);
        return 1;
    }

    printf("%-16s %-9s %9s %9s %9s %9s %9s\n", "scenario", "mode", "end_pos",
           "max_pos", "max_speed", "max_duty", "fault_at");
    for (i = 0; i < count; i++) {
        sim_result_t r = sim_run(&sc[i], &cfg, NULL);

        printf("%-16s %-9s %9.2f %9.2f %9.1f %9.2f %9.2f\n", sc[i].name,
               sim_mode_name(r.final_mode), (double)r.final_pos_mm,
               (double)r.max_pos_mm, (double)r.max_speed_mm_s,
               (double)r.max_abs_duty, (double)r.time_to_fault_s);
    }
    return 0;
}
