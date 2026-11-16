/**
 * @file    can_messages.h
 * @brief   The vehicle CAN "database": message IDs, signal layout, packing
 *          and unpacking, rolling counters and checksums.
 *
 * Header-only and plain C, so the SAME file is used by:
 *   - the STM32 Engine ECU   (firmware/stm32_engine_ecu, via include path)
 *   - the ESP32-S3 Dashboard (copied into the sketch folder; CI checks the
 *                             two copies are identical)
 * A matching DBC file for tools like SavvyCAN is in docs/vehicle.dbc.
 *
 * Bus: classic CAN 2.0A (11-bit IDs), 125 kbit/s, Intel byte order (LSB first).
 * Lower ID = higher priority in arbitration, so the most important
 * message (engine data) gets the lowest ID.
 *
 * ---------------------------------------------------------------------------
 * 0x0C0 ENGINE_STATUS   ECU -> all   DLC 8   every 100 ms
 *   byte 0-1  rpm            uint16   1 rpm/bit
 *   byte 2    speed          uint8    1 km/h/bit
 *   byte 3    coolant temp   uint8    1 C/bit, offset -40  (0 = -40 C)
 *   byte 4    throttle       uint8    1 %/bit
 *   byte 5    gear           uint8    0 = neutral, 1..6
 *   byte 6    bits 0-3 rolling counter (0..15), bit 4 check-engine lamp
 *   byte 7    checksum
 *
 * 0x3C0 BODY_STATUS     ECU -> all   DLC 8   every 500 ms
 *   byte 0-1  battery        uint16   1 mV/bit
 *   byte 2    fuel level     uint8    1 %/bit
 *   byte 3    bit 0 ignition on, bit 1 door open, bit 2 seat belt fastened
 *   byte 4-5  reserved (0)
 *   byte 6    bits 0-3 rolling counter
 *   byte 7    checksum
 *
 * 0x2A0 DASH_STATUS     Dashboard -> ECU   DLC 8   every 200 ms
 *   byte 0    warning flags  (DASH_WARN_* below)
 *   byte 1    frames received OK (low byte, wraps)
 *   byte 2    frames rejected (checksum/counter errors, low byte)
 *   byte 3-5  reserved (0)
 *   byte 6    bits 0-3 rolling counter
 *   byte 7    checksum
 * ---------------------------------------------------------------------------
 *
 * Checksum (similar in spirit to AUTOSAR E2E protection):
 *   checksum = 0xFF XOR (low byte of ID + high byte of ID + data[0..6]) mod 256
 * Including the ID means a frame that ends up under the wrong ID is caught.
 * The rolling counter detects lost, repeated or frozen frames.
 */
#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ IDs */
#define CAN_ID_ENGINE_STATUS 0x0C0u
#define CAN_ID_DASH_STATUS   0x2A0u
#define CAN_ID_BODY_STATUS   0x3C0u

#define CAN_PERIOD_ENGINE_MS 100u
#define CAN_PERIOD_DASH_MS   200u
#define CAN_PERIOD_BODY_MS   500u

/* A message is "lost" if it is missing for this many periods. */
#define CAN_TIMEOUT_PERIODS  5u

/* ----------------------------------------------------- warnings */
#define DASH_WARN_OVERHEAT   (1u << 0) /* coolant > 105 C          */
#define DASH_WARN_OVERSPEED  (1u << 1) /* speed > 130 km/h         */
#define DASH_WARN_LOW_FUEL   (1u << 2) /* fuel < 10 %              */
#define DASH_WARN_LOW_BATT   (1u << 3) /* battery < 11.8 V         */
#define DASH_WARN_ENGINE_TO  (1u << 4) /* ENGINE_STATUS timed out  */
#define DASH_WARN_BODY_TO    (1u << 5) /* BODY_STATUS timed out    */
#define DASH_WARN_E2E_ERROR  (1u << 6) /* checksum/counter errors  */
#define DASH_WARN_CHECK_ENG  (1u << 7) /* ECU set check-engine     */

/* --------------------------------------------- decoded signals */
typedef struct {
    uint16_t rpm;
    uint8_t  speed_kmh;
    int16_t  coolant_c;
    uint8_t  throttle_pct;
    uint8_t  gear;
    bool     check_engine;
    uint8_t  counter;
} engine_status_t;

typedef struct {
    uint16_t battery_mv;
    uint8_t  fuel_pct;
    bool     ignition_on;
    bool     door_open;
    bool     belt_fastened;
    uint8_t  counter;
} body_status_t;

typedef struct {
    uint8_t warnings;
    uint8_t rx_ok;
    uint8_t rx_rejected;
    uint8_t counter;
} dash_status_t;

/* ---------------------------------------------- E2E protection */
static inline uint8_t can_checksum(uint16_t id, const uint8_t d[8])
{
    uint8_t sum = (uint8_t)((id & 0xFFu) + (id >> 8));
    for (int i = 0; i < 7; i++) {
        sum = (uint8_t)(sum + d[i]);
    }
    return (uint8_t)(sum ^ 0xFFu);
}

static inline bool can_checksum_ok(uint16_t id, const uint8_t d[8])
{
    return can_checksum(id, d) == d[7];
}

/** True if `now` is the expected successor of `prev` (4-bit counter). */
static inline bool can_counter_ok(uint8_t prev, uint8_t now)
{
    return (uint8_t)((prev + 1u) & 0x0Fu) == (now & 0x0Fu);
}

/* ----------------------------------------------- ENGINE_STATUS */
static inline void engine_status_pack(const engine_status_t *s, uint8_t d[8])
{
    d[0] = (uint8_t)(s->rpm & 0xFFu);
    d[1] = (uint8_t)(s->rpm >> 8);
    d[2] = s->speed_kmh;
    d[3] = (uint8_t)(s->coolant_c + 40);
    d[4] = s->throttle_pct;
    d[5] = s->gear;
    d[6] = (uint8_t)((s->counter & 0x0Fu) | (s->check_engine ? 0x10u : 0u));
    d[7] = can_checksum(CAN_ID_ENGINE_STATUS, d);
}

static inline void engine_status_unpack(const uint8_t d[8], engine_status_t *s)
{
    s->rpm          = (uint16_t)(d[0] | (d[1] << 8));
    s->speed_kmh    = d[2];
    s->coolant_c    = (int16_t)(d[3] - 40);
    s->throttle_pct = d[4];
    s->gear         = d[5];
    s->counter      = d[6] & 0x0Fu;
    s->check_engine = (d[6] & 0x10u) != 0u;
}

/* ------------------------------------------------- BODY_STATUS */
static inline void body_status_pack(const body_status_t *s, uint8_t d[8])
{
    d[0] = (uint8_t)(s->battery_mv & 0xFFu);
    d[1] = (uint8_t)(s->battery_mv >> 8);
    d[2] = s->fuel_pct;
    d[3] = (uint8_t)((s->ignition_on ? 1u : 0u) | (s->door_open ? 2u : 0u) |
                     (s->belt_fastened ? 4u : 0u));
    d[4] = 0;
    d[5] = 0;
    d[6] = s->counter & 0x0Fu;
    d[7] = can_checksum(CAN_ID_BODY_STATUS, d);
}

static inline void body_status_unpack(const uint8_t d[8], body_status_t *s)
{
    s->battery_mv    = (uint16_t)(d[0] | (d[1] << 8));
    s->fuel_pct      = d[2];
    s->ignition_on   = (d[3] & 1u) != 0u;
    s->door_open     = (d[3] & 2u) != 0u;
    s->belt_fastened = (d[3] & 4u) != 0u;
    s->counter       = d[6] & 0x0Fu;
}

/* ------------------------------------------------- DASH_STATUS */
static inline void dash_status_pack(const dash_status_t *s, uint8_t d[8])
{
    d[0] = s->warnings;
    d[1] = s->rx_ok;
    d[2] = s->rx_rejected;
    d[3] = 0;
    d[4] = 0;
    d[5] = 0;
    d[6] = s->counter & 0x0Fu;
    d[7] = can_checksum(CAN_ID_DASH_STATUS, d);
}

static inline void dash_status_unpack(const uint8_t d[8], dash_status_t *s)
{
    s->warnings    = d[0];
    s->rx_ok       = d[1];
    s->rx_rejected = d[2];
    s->counter     = d[6] & 0x0Fu;
}

#ifdef __cplusplus
}
#endif

#endif /* CAN_MESSAGES_H */
