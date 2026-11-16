/**
 * Host unit tests for firmware/common/can_messages.h.
 * Runs on your Mac/PC (no hardware):  make -C tests
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "can_messages.h"

static int passed;
#define CHECK(cond) do { assert(cond); passed++; } while (0)

static void test_engine_roundtrip(void)
{
    engine_status_t in = { .rpm = 3456, .speed_kmh = 87, .coolant_c = -12, .throttle_pct = 42,
                           .gear = 5, .check_engine = true, .counter = 13 }, out;
    uint8_t d[8];
    engine_status_pack(&in, d);
    CHECK(can_checksum_ok(CAN_ID_ENGINE_STATUS, d));
    engine_status_unpack(d, &out);
    CHECK(out.rpm == 3456 && out.speed_kmh == 87 && out.coolant_c == -12);
    CHECK(out.throttle_pct == 42 && out.gear == 5 && out.check_engine && out.counter == 13);
    CHECK(d[0] == (3456 & 0xFF) && d[1] == (3456 >> 8)); /* Intel byte order */
}

static void test_body_roundtrip(void)
{
    body_status_t in = { .battery_mv = 12650, .fuel_pct = 7, .ignition_on = true,
                         .door_open = true, .belt_fastened = false, .counter = 15 }, out;
    uint8_t d[8];
    body_status_pack(&in, d);
    CHECK(can_checksum_ok(CAN_ID_BODY_STATUS, d));
    body_status_unpack(d, &out);
    CHECK(out.battery_mv == 12650 && out.fuel_pct == 7 && out.ignition_on);
    CHECK(out.door_open && !out.belt_fastened && out.counter == 15);
}

static void test_checksum_detects_corruption(void)
{
    dash_status_t s = { .warnings = 0x81, .rx_ok = 200, .rx_rejected = 3, .counter = 4 };
    uint8_t d[8];
    dash_status_pack(&s, d);
    CHECK(can_checksum_ok(CAN_ID_DASH_STATUS, d));
    d[2] ^= 0x01;                                   /* single bit flip */
    CHECK(!can_checksum_ok(CAN_ID_DASH_STATUS, d));
    d[2] ^= 0x01;
    CHECK(!can_checksum_ok(CAN_ID_ENGINE_STATUS, d)); /* right data, wrong ID */
}

static void test_counter(void)
{
    CHECK(can_counter_ok(0, 1));
    CHECK(can_counter_ok(15, 0));   /* wraps at 16 */
    CHECK(!can_counter_ok(3, 3));   /* frozen      */
    CHECK(!can_counter_ok(3, 5));   /* frame lost  */
}

int main(void)
{
    test_engine_roundtrip();
    test_body_roundtrip();
    test_checksum_detects_corruption();
    test_counter();
    printf("All %d checks passed\n", passed);
    return 0;
}
