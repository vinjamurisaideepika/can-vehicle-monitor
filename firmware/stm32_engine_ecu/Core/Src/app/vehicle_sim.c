/**
 * @file    vehicle_sim.c
 * @brief   Integer-only vehicle model, stepped every 100 ms.
 *
 * Fixed-point units keep the maths fast and printf float-free:
 *   speed in 0.1 km/h, coolant in 0.01 C, fuel in 0.01 %.
 */
#include "app/vehicle_sim.h"

/* rpm per 1 km/h in each gear (index 0 = neutral). */
static const uint16_t k_rpm_per_kmh[7] = { 0, 120, 70, 48, 37, 30, 25 };

static drive_mode_t s_mode;
static int32_t  s_speed_d;    /* 0.1 km/h */
static int32_t  s_coolant_c;  /* 0.01 C   */
static int32_t  s_fuel_c;     /* 0.01 %   */
static uint16_t s_rpm;
static uint8_t  s_gear;
static uint8_t  s_throttle;
static bool     s_door_open;
static uint32_t s_noise = 12345u;

/* Tiny pseudo-random generator for sensor noise (xorshift32). */
static int32_t noise(int32_t amplitude)
{
    s_noise ^= s_noise << 13;
    s_noise ^= s_noise >> 17;
    s_noise ^= s_noise << 5;
    return (int32_t)(s_noise % (uint32_t)(2 * amplitude + 1)) - amplitude;
}

static uint8_t gear_for_speed(int32_t kmh)
{
    if (kmh < 15) return 1;
    if (kmh < 30) return 2;
    if (kmh < 50) return 3;
    if (kmh < 70) return 4;
    if (kmh < 95) return 5;
    return 6;
}

void vehicle_sim_init(void)
{
    s_mode      = DRIVE_IDLE;
    s_speed_d   = 0;
    s_coolant_c = 2000; /* cold start: 20 C */
    s_fuel_c    = 3500; /* 35 % */
    s_door_open = false;
}

void vehicle_sim_step_100ms(void)
{
    /* 1) Speed follows the driver's input. */
    switch (s_mode) {
    case DRIVE_ACCEL:  s_speed_d += 15; s_throttle = 70; break; /* +1.5 km/h */
    case DRIVE_CRUISE: s_throttle = 25;                  break;
    case DRIVE_BRAKE:  s_speed_d -= 30; s_throttle = 0;  break; /* -3 km/h   */
    default:           s_speed_d -= 3;  s_throttle = 0;  break; /* coasting  */
    }
    if (s_speed_d < 0)    s_speed_d = 0;
    if (s_speed_d > 1800) s_speed_d = 1800; /* top speed 180 km/h */

    /* 2) Gearbox and engine speed. */
    int32_t kmh = s_speed_d / 10;
    if (kmh == 0 && s_mode != DRIVE_ACCEL) {
        s_gear = 0;
        s_rpm  = (uint16_t)(800 + noise(20)); /* idle */
    } else {
        s_gear = gear_for_speed(kmh);
        int32_t rpm = kmh * k_rpm_per_kmh[s_gear] + noise(30);
        if (rpm < 900)  rpm = 900;
        if (rpm > 6500) rpm = 6500;
        s_rpm = (uint16_t)rpm;
    }

    /* 3) Coolant moves slowly toward a target that depends on load:
     *    hard acceleration above 100 km/h overheats the engine. */
    bool heavy_load = (s_mode == DRIVE_ACCEL) && (kmh > 100);
    int32_t target = heavy_load ? 11200 : 9000;
    s_coolant_c += (target - s_coolant_c) / 100; /* ~10 s time constant */

    /* 4) Fuel burns faster with more throttle. */
    s_fuel_c -= 1 + s_throttle / 10;
    if (s_fuel_c < 0) s_fuel_c = 0;
}

void         vehicle_sim_set_mode(drive_mode_t m) { if (m < DRIVE_MODE_COUNT) s_mode = m; }
drive_mode_t vehicle_sim_get_mode(void)           { return s_mode; }
void         vehicle_sim_set_fuel(uint8_t pct)    { s_fuel_c = (pct > 100 ? 100 : pct) * 100; }
void         vehicle_sim_set_door(bool open)      { s_door_open = open; }

const char *vehicle_sim_mode_name(drive_mode_t m)
{
    static const char *const names[] = { "idle", "accel", "cruise", "brake" };
    return (m < DRIVE_MODE_COUNT) ? names[m] : "?";
}

void vehicle_sim_get_engine(engine_status_t *out)
{
    out->rpm          = s_rpm;
    out->speed_kmh    = (uint8_t)(s_speed_d / 10);
    out->coolant_c    = (int16_t)(s_coolant_c / 100);
    out->throttle_pct = s_throttle;
    out->gear         = s_gear;
    out->check_engine = s_coolant_c > 11000; /* engine protection */
}

void vehicle_sim_get_body(body_status_t *out)
{
    /* Alternator charges at ~14.1 V when the engine turns. */
    out->battery_mv    = (uint16_t)(14100 + noise(40));
    out->fuel_pct      = (uint8_t)(s_fuel_c / 100);
    out->ignition_on   = true;
    out->door_open     = s_door_open;
    out->belt_fastened = true;
}
