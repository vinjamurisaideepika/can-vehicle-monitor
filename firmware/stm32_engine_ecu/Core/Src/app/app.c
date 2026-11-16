/**
 * @file    app.c
 * @brief   Engine ECU node: periodic CAN transmission, receive handling,
 *          node state machine and UART diagnostic console.
 *
 * Scheduling: TIM6 interrupts at 100 Hz and only increments a counter.
 * The main loop turns elapsed ticks into periodic jobs:
 *
 *   every 10 ms  : node state machine, drain CAN RX queue, console
 *   every 100 ms : vehicle model step + ENGINE_STATUS (0x0C0)
 *   every 500 ms : BODY_STATUS (0x3C0)
 *   every 1 s    : monitor line on UART (if enabled)
 *
 * Node state machine:
 *
 *        +------+  CAN started   +---------+  no DASH_STATUS   +----------+
 *        | INIT | -------------> | RUNNING | ----------------> | DEGRADED |
 *        +------+                +---------+ <---------------- +----------+
 *                                     |       DASH_STATUS back       |
 *                     bus-off (TEC>255)|                              | bus-off
 *                                     v                              v
 *                               +---------+   wait 1 s   +------------+
 *                               | BUS_OFF | -----------> | RECOVERING |--> RUNNING
 *                               +---------+              +------------+    (or DEGRADED)
 */
#include "app/app.h"
#include "app/can_driver.h"
#include "app/uart_console.h"
#include "app/vehicle_sim.h"
#include "can_messages.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern CAN_HandleTypeDef  hcan1;
extern TIM_HandleTypeDef  htim6;
extern UART_HandleTypeDef huart2;

#define FW_VERSION       "1.0.0"
#define TICK_MS          10u
#define BUSOFF_WAIT_MS   1000u
#define DASH_TIMEOUT_MS  (CAN_PERIOD_DASH_MS * CAN_TIMEOUT_PERIODS)
#define FAULT_FRAMES     10u

/* -------------------------------------------------------- state machine */
typedef enum {
    NODE_INIT = 0,
    NODE_RUNNING,
    NODE_DEGRADED,
    NODE_BUS_OFF,
    NODE_RECOVERING
} node_state_t;

static const char *const k_state_names[] = {
    "INIT", "RUNNING", "DEGRADED", "BUS_OFF", "RECOVERING"
};

static node_state_t s_state = NODE_INIT;
static uint32_t     s_state_entered_ms;

/* ---------------------------------------------------------- variables */
static volatile uint32_t s_ticks;      /* TIM6 100 Hz */
static volatile bool     s_button;     /* B1 pressed  */
static uint32_t s_last_tick;

static uint8_t s_engine_counter, s_body_counter;
static bool    s_tx_muted;
static bool    s_log_frames;
static bool    s_monitor = true;
static uint8_t s_fault_crc_left, s_fault_counter_left;

/* Dashboard supervision */
static bool          s_dash_seen;
static uint32_t      s_dash_last_ms;
static uint8_t       s_dash_last_counter;
static dash_status_t s_dash;
static uint32_t      s_dash_e2e_errors;

/* ======================================================= state machine */
static void enter_state(node_state_t next)
{
    if (next == s_state) {
        return;
    }
    printf("[state] %s -> %s\r\n", k_state_names[s_state], k_state_names[next]);
    s_state = next;
    s_state_entered_ms = HAL_GetTick();
}

static bool dash_alive(uint32_t now)
{
    return s_dash_seen && (now - s_dash_last_ms) < DASH_TIMEOUT_MS;
}

static void state_machine_run(void)
{
    uint32_t now = HAL_GetTick();
    bool bus_off = (can_drv_bus_state() == CAN_BUS_OFF);

    switch (s_state) {
    case NODE_INIT:
        enter_state(NODE_DEGRADED); /* until we hear the dashboard */
        break;

    case NODE_RUNNING:
        if (bus_off) {
            enter_state(NODE_BUS_OFF);
        } else if (!dash_alive(now)) {
            printf("[warn] DASH_STATUS timeout (> %lu ms)\r\n", (unsigned long)DASH_TIMEOUT_MS);
            enter_state(NODE_DEGRADED);
        }
        break;

    case NODE_DEGRADED:
        if (bus_off) {
            enter_state(NODE_BUS_OFF);
        } else if (dash_alive(now)) {
            enter_state(NODE_RUNNING);
        }
        break;

    case NODE_BUS_OFF:
        /* Stay quiet for a while before trying to rejoin the bus. */
        if (now - s_state_entered_ms >= BUSOFF_WAIT_MS) {
            printf("[info] attempting bus-off recovery\r\n");
            can_drv_recover();
            enter_state(NODE_RECOVERING);
        }
        break;

    case NODE_RECOVERING:
        if (bus_off) {
            enter_state(NODE_BUS_OFF);
        } else if (now - s_state_entered_ms > 200u) {
            enter_state(dash_alive(now) ? NODE_RUNNING : NODE_DEGRADED);
        }
        break;
    }
}

static bool tx_allowed(void)
{
    return !s_tx_muted && (s_state == NODE_RUNNING || s_state == NODE_DEGRADED);
}

/* ========================================================= transmit */
static void send_engine_status(void)
{
    engine_status_t e;
    uint8_t d[8];
    vehicle_sim_get_engine(&e);

    /* Fault injection: freeze the counter to test the receiver. */
    if (s_fault_counter_left > 0u) {
        s_fault_counter_left--;
    } else {
        s_engine_counter = (uint8_t)((s_engine_counter + 1u) & 0x0Fu);
    }
    e.counter = s_engine_counter;
    engine_status_pack(&e, d);

    if (s_fault_crc_left > 0u) { /* corrupt the checksum on purpose */
        s_fault_crc_left--;
        d[7] ^= 0x5A;
    }

    if (tx_allowed()) {
        can_drv_send(CAN_ID_ENGINE_STATUS, d, 8);
        if (s_log_frames) {
            printf("TX 0x%03X rpm=%u spd=%u gear=%u cool=%d ctr=%u\r\n",
                   CAN_ID_ENGINE_STATUS, e.rpm, e.speed_kmh, e.gear, e.coolant_c, e.counter);
        }
    }
}

static void send_body_status(void)
{
    body_status_t b;
    uint8_t d[8];
    vehicle_sim_get_body(&b);
    s_body_counter = (uint8_t)((s_body_counter + 1u) & 0x0Fu);
    b.counter = s_body_counter;
    body_status_pack(&b, d);
    if (tx_allowed()) {
        can_drv_send(CAN_ID_BODY_STATUS, d, 8);
    }
}

/* ========================================================== receive */
static void print_warnings(uint8_t w)
{
    if (w == 0u) { printf(" none"); return; }
    if (w & DASH_WARN_OVERHEAT)  printf(" OVERHEAT");
    if (w & DASH_WARN_OVERSPEED) printf(" OVERSPEED");
    if (w & DASH_WARN_LOW_FUEL)  printf(" LOW_FUEL");
    if (w & DASH_WARN_LOW_BATT)  printf(" LOW_BATT");
    if (w & DASH_WARN_ENGINE_TO) printf(" ENGINE_TIMEOUT");
    if (w & DASH_WARN_BODY_TO)   printf(" BODY_TIMEOUT");
    if (w & DASH_WARN_E2E_ERROR) printf(" E2E_ERROR");
    if (w & DASH_WARN_CHECK_ENG) printf(" CHECK_ENGINE");
}

static void handle_frame(const can_frame_t *f)
{
    if (s_log_frames) {
        printf("RX 0x%03lX [%u]", (unsigned long)f->id, f->dlc);
        for (uint8_t i = 0; i < f->dlc; i++) printf(" %02X", f->data[i]);
        printf("\r\n");
    }

    if (f->id != CAN_ID_DASH_STATUS) {
        return; /* in loopback mode we also receive our own frames */
    }
    if (f->dlc != 8u || !can_checksum_ok(CAN_ID_DASH_STATUS, f->data)) {
        s_dash_e2e_errors++;
        return;
    }

    dash_status_t d;
    dash_status_unpack(f->data, &d);
    if (s_dash_seen && !can_counter_ok(s_dash_last_counter, d.counter)) {
        s_dash_e2e_errors++; /* lost or repeated frame - data still usable */
    }
    if (!s_dash_seen || d.warnings != s_dash.warnings) {
        printf("[dash] warnings:");
        print_warnings(d.warnings);
        printf("\r\n");
    }
    s_dash = d;
    s_dash_seen = true;
    s_dash_last_counter = d.counter;
    s_dash_last_ms = f->timestamp_ms;
}

/* ========================================================== console */
static void print_status(void)
{
    can_stats_t st;
    can_drv_get_stats(&st);
    static const char *const bus_names[] = { "error-active", "error-warning", "error-passive", "bus-off" };

    printf("FW %s | state %s | drive %s | tx %s\r\n", FW_VERSION, k_state_names[s_state],
           vehicle_sim_mode_name(vehicle_sim_get_mode()), s_tx_muted ? "MUTED" : "on");
    printf("CAN  %s TEC=%u REC=%u last-error=%s\r\n", bus_names[can_drv_bus_state()],
           st.tec, st.rec, can_drv_lec_name(st.last_error));
    printf("     tx_ok=%lu tx_fail=%lu rx_ok=%lu rx_ovf=%lu err_irq=%lu bus_off=%lu\r\n",
           (unsigned long)st.tx_ok, (unsigned long)st.tx_fail, (unsigned long)st.rx_ok,
           (unsigned long)st.rx_overflow, (unsigned long)st.err_irq,
           (unsigned long)st.bus_off_events);
    printf("DASH %s, e2e errors=%lu, reports rx_ok=%u rejected=%u, warnings:",
           dash_alive(HAL_GetTick()) ? "alive" : "TIMEOUT", (unsigned long)s_dash_e2e_errors,
           s_dash.rx_ok, s_dash.rx_rejected);
    print_warnings(s_dash.warnings);
    printf("\r\n");
}

static void print_help(void)
{
    printf("Commands:\r\n"
           "  help | status\r\n"
           "  drive idle|accel|cruise|brake   (button B1 cycles too)\r\n"
           "  fuel <0-100>     set fuel level\r\n"
           "  door open|close\r\n"
           "  log on|off       print every CAN frame\r\n"
           "  monitor on|off   1 Hz summary line\r\n"
           "  mute on|off      stop transmitting (dashboard sees timeout)\r\n"
           "  fault crc        send 10 frames with a bad checksum\r\n"
           "  fault counter    freeze the rolling counter for 10 frames\r\n");
}

static bool arg_on(const char *a) { return a != NULL && strcmp(a, "on") == 0; }

static void handle_command(char *line)
{
    char *cmd = strtok(line, " ");
    char *arg = strtok(NULL, " ");
    if (cmd == NULL) return;

    if (strcmp(cmd, "help") == 0) {
        print_help();
    } else if (strcmp(cmd, "status") == 0) {
        print_status();
    } else if (strcmp(cmd, "drive") == 0 && arg != NULL) {
        for (int m = 0; m < DRIVE_MODE_COUNT; m++) {
            if (strcmp(arg, vehicle_sim_mode_name((drive_mode_t)m)) == 0) {
                vehicle_sim_set_mode((drive_mode_t)m);
                printf("drive mode: %s\r\n", arg);
                return;
            }
        }
        printf("error: idle|accel|cruise|brake\r\n");
    } else if (strcmp(cmd, "fuel") == 0 && arg != NULL) {
        vehicle_sim_set_fuel((uint8_t)atoi(arg));
        printf("fuel set\r\n");
    } else if (strcmp(cmd, "door") == 0 && arg != NULL) {
        vehicle_sim_set_door(strcmp(arg, "open") == 0);
        printf("door %s\r\n", arg);
    } else if (strcmp(cmd, "log") == 0) {
        s_log_frames = arg_on(arg);
    } else if (strcmp(cmd, "monitor") == 0) {
        s_monitor = arg_on(arg);
    } else if (strcmp(cmd, "mute") == 0) {
        s_tx_muted = arg_on(arg);
        printf("transmit %s\r\n", s_tx_muted ? "muted" : "enabled");
    } else if (strcmp(cmd, "fault") == 0 && arg != NULL && strcmp(arg, "crc") == 0) {
        s_fault_crc_left = FAULT_FRAMES;
        printf("injecting %u bad checksums\r\n", FAULT_FRAMES);
    } else if (strcmp(cmd, "fault") == 0 && arg != NULL && strcmp(arg, "counter") == 0) {
        s_fault_counter_left = FAULT_FRAMES;
        printf("freezing counter for %u frames\r\n", FAULT_FRAMES);
    } else {
        printf("unknown command '%s' - type 'help'\r\n", cmd);
    }
}

static void print_monitor_line(void)
{
    engine_status_t e;
    can_stats_t st;
    vehicle_sim_get_engine(&e);
    can_drv_get_stats(&st);
    printf("[%-10s] %-6s spd=%3u km/h rpm=%4u gear=%u cool=%3d C | tx=%lu rx=%lu TEC=%u REC=%u\r\n",
           k_state_names[s_state], vehicle_sim_mode_name(vehicle_sim_get_mode()),
           e.speed_kmh, e.rpm, e.gear, e.coolant_c,
           (unsigned long)st.tx_ok, (unsigned long)st.rx_ok, st.tec, st.rec);
}

static void update_led(uint32_t ticks)
{
    GPIO_PinState on;
    switch (s_state) {
    case NODE_RUNNING:  on = ((ticks / 50u) % 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET; break; /* 1 Hz */
    case NODE_DEGRADED: on = ((ticks / 10u) % 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET; break; /* 5 Hz */
    case NODE_BUS_OFF:  on = GPIO_PIN_SET;   break;                                      /* solid */
    default:            on = GPIO_PIN_RESET; break;
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on);
}

/* ============================================================ public */
void app_init(void)
{
    uart_console_init(&huart2);
    printf("\r\n=== CAN Vehicle Monitor - Engine ECU node v%s ===\r\n", FW_VERSION);

    vehicle_sim_init();
    if (can_drv_init(&hcan1) != HAL_OK) {
        printf("[error] CAN init failed - check CubeMX CAN1 settings\r\n");
    } else {
        printf("CAN1 started (125 kbit/s, PA11=RX PA12=TX)\r\n");
    }
    print_help();
    s_state_entered_ms = HAL_GetTick();
    HAL_TIM_Base_Start_IT(&htim6);
}

void app_loop(void)
{
    char line[CONSOLE_LINE_MAX];
    if (uart_console_get_line(line, sizeof line)) {
        handle_command(line);
    }

    if (s_button) {
        s_button = false;
        drive_mode_t next = (drive_mode_t)((vehicle_sim_get_mode() + 1) % DRIVE_MODE_COUNT);
        vehicle_sim_set_mode(next);
        printf("[button] drive mode: %s\r\n", vehicle_sim_mode_name(next));
    }

    can_frame_t f;
    while (can_drv_receive(&f)) {
        handle_frame(&f);
    }

    uint32_t now = s_ticks;
    while (s_last_tick != now) { /* run every 10 ms job exactly once per tick */
        s_last_tick++;
        state_machine_run();
        update_led(s_last_tick);

        if (s_last_tick % (CAN_PERIOD_ENGINE_MS / TICK_MS) == 0u) {
            vehicle_sim_step_100ms();
            send_engine_status();
        }
        if (s_last_tick % (CAN_PERIOD_BODY_MS / TICK_MS) == 0u) {
            send_body_status();
        }
        if (s_monitor && !s_log_frames && s_last_tick % 100u == 0u) {
            print_monitor_line();
        }
    }
}

/* ===================================================== HAL callbacks */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        s_ticks++;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_ms;
    if (GPIO_Pin == B1_Pin && HAL_GetTick() - last_ms > 200u) {
        last_ms = HAL_GetTick();
        s_button = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { uart_console_rx_isr(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { uart_console_error_isr(huart); }
