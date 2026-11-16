/**
 * @file    vehicle_sim.h
 * @brief   Very small vehicle model that produces realistic-looking signals
 *          (speed, rpm, gear, coolant temperature, fuel, battery).
 */
#ifndef APP_VEHICLE_SIM_H
#define APP_VEHICLE_SIM_H

#include "can_messages.h"

typedef enum { DRIVE_IDLE = 0, DRIVE_ACCEL, DRIVE_CRUISE, DRIVE_BRAKE, DRIVE_MODE_COUNT } drive_mode_t;

void         vehicle_sim_init(void);
void         vehicle_sim_step_100ms(void); /* advance the model by 100 ms */
void         vehicle_sim_set_mode(drive_mode_t m);
drive_mode_t vehicle_sim_get_mode(void);
const char  *vehicle_sim_mode_name(drive_mode_t m);
void         vehicle_sim_set_fuel(uint8_t pct);
void         vehicle_sim_set_door(bool open);

void vehicle_sim_get_engine(engine_status_t *out); /* counter not filled */
void vehicle_sim_get_body(body_status_t *out);     /* counter not filled */

#endif /* APP_VEHICLE_SIM_H */
