/**
 * CAN Vehicle Monitor - Dashboard node (ESP32-S3, built-in TWAI controller).
 *
 * "TWAI" (Two-Wire Automotive Interface) is Espressif's name for its
 * classic CAN 2.0 controller. Like any CAN controller it needs a
 * transceiver - or, on a breadboard, the diode wired-AND bus described in
 * docs/wiring.md.
 *
 * What this node does:
 *   - receives ENGINE_STATUS (0x0C0) and BODY_STATUS (0x3C0) from the ECU
 *   - checks DLC, checksum and rolling counter of every frame (E2E checks)
 *   - detects message timeouts
 *   - computes warnings and sends DASH_STATUS (0x2A0) every 200 ms
 *   - handles bus errors / bus-off with automatic recovery
 *   - prints a text "instrument cluster" on the Serial Monitor
 *   - RGB LED: green = OK, yellow = warning, red = timeout / bus problem
 *
 * Board : ESP32-S3-DevKitC-1, Arduino-ESP32 core 3.x, Serial Monitor 115200
 * Pins  : GPIO4 = TWAI TX, GPIO5 = TWAI RX
 */
#include <Arduino.h>
#include "driver/twai.h"
#include "can_messages.h"  // identical copy of firmware/common/can_messages.h

static const gpio_num_t PIN_CAN_TX = GPIO_NUM_4;
static const gpio_num_t PIN_CAN_RX = GPIO_NUM_5;

// Thresholds that raise warnings
static const int OVERHEAT_C = 105;
static const int OVERSPEED_KMH = 130;
static const int LOW_FUEL_PCT = 10;
static const int LOW_BATT_MV = 11800;

// ----------------------------------------------------------- state
struct RxTrack {
  bool seen = false;
  uint32_t lastMs = 0;
  uint8_t lastCounter = 0;
};

static RxTrack engTrack, bodyTrack;
static engine_status_t engine = {};
static body_status_t body = {};
static uint32_t rxOk = 0, rxRejected = 0, e2eErrorsRecent = 0;
static uint8_t warnings = 0, dashCounter = 0;
static bool busOff = false;
static bool logFrames = false;

// ======================================================== TWAI setup
static void canStart() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 20;
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();  // must match the STM32
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());
  Serial.println("TWAI started: 125 kbit/s, TX=GPIO4 RX=GPIO5");
}

// ============================================================ receive
// Returns true if the frame passes the end-to-end checks.
static bool e2eCheck(const twai_message_t &m, RxTrack &trk) {
  if (m.data_length_code != 8 || !can_checksum_ok(m.identifier, m.data)) {
    rxRejected++;
    e2eErrorsRecent++;
    Serial.printf("[e2e] 0x%03lX rejected: bad DLC/checksum\n", (unsigned long)m.identifier);
    return false;
  }
  uint8_t ctr = m.data[6] & 0x0F;
  if (trk.seen && !can_counter_ok(trk.lastCounter, ctr)) {
    e2eErrorsRecent++;  // a lost or repeated frame: note it, data still valid
    Serial.printf("[e2e] 0x%03lX counter jump %u -> %u\n", (unsigned long)m.identifier,
                  trk.lastCounter, ctr);
  }
  trk.seen = true;
  trk.lastCounter = ctr;
  trk.lastMs = millis();
  rxOk++;
  return true;
}

static void handleFrame(const twai_message_t &m) {
  if (logFrames) {
    Serial.printf("RX 0x%03lX [%u]", (unsigned long)m.identifier, m.data_length_code);
    for (int i = 0; i < m.data_length_code; i++) Serial.printf(" %02X", m.data[i]);
    Serial.println();
  }
  if (m.extd || m.rtr) return;  // we only use standard data frames

  switch (m.identifier) {
    case CAN_ID_ENGINE_STATUS:
      if (e2eCheck(m, engTrack)) engine_status_unpack(m.data, &engine);
      break;
    case CAN_ID_BODY_STATUS:
      if (e2eCheck(m, bodyTrack)) body_status_unpack(m.data, &body);
      break;
    default:
      break;  // unknown IDs are ignored
  }
}

// ============================================================ logic
static bool timedOut(const RxTrack &t, uint32_t periodMs) {
  return !t.seen || (millis() - t.lastMs) > periodMs * CAN_TIMEOUT_PERIODS;
}

static uint8_t computeWarnings() {
  uint8_t w = 0;
  bool engOk = !timedOut(engTrack, CAN_PERIOD_ENGINE_MS);
  bool bodyOk = !timedOut(bodyTrack, CAN_PERIOD_BODY_MS);
  if (!engOk) w |= DASH_WARN_ENGINE_TO;
  if (!bodyOk) w |= DASH_WARN_BODY_TO;
  if (engOk && engine.coolant_c > OVERHEAT_C) w |= DASH_WARN_OVERHEAT;
  if (engOk && engine.speed_kmh > OVERSPEED_KMH) w |= DASH_WARN_OVERSPEED;
  if (engOk && engine.check_engine) w |= DASH_WARN_CHECK_ENG;
  if (bodyOk && body.fuel_pct < LOW_FUEL_PCT) w |= DASH_WARN_LOW_FUEL;
  if (bodyOk && body.battery_mv < LOW_BATT_MV) w |= DASH_WARN_LOW_BATT;
  if (e2eErrorsRecent > 0) w |= DASH_WARN_E2E_ERROR;
  return w;
}

static void sendDashStatus() {
  if (busOff) return;
  dash_status_t d = {};
  d.warnings = warnings;
  d.rx_ok = (uint8_t)rxOk;
  d.rx_rejected = (uint8_t)rxRejected;
  d.counter = dashCounter = (dashCounter + 1) & 0x0F;

  twai_message_t m = {};
  m.identifier = CAN_ID_DASH_STATUS;
  m.data_length_code = 8;
  dash_status_pack(&d, m.data);
  // Don't block: if the TX queue is full (nobody ACKs), just drop it.
  twai_transmit(&m, 0);
}

static void handleAlerts() {
  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) != ESP_OK) return;

  if (alerts & TWAI_ALERT_ERR_PASS) Serial.println("[can] error-passive");
  if (alerts & TWAI_ALERT_ERR_ACTIVE) Serial.println("[can] error-active again");
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.println("[can] RX queue full");
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.println("[can] BUS-OFF -> starting recovery");
    busOff = true;
    twai_initiate_recovery();  // waits for 128 x 11 recessive bits
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    Serial.println("[can] recovered -> restarting");
    twai_start();
    busOff = false;
  }
}

static void setLed() {
  if (busOff || (warnings & (DASH_WARN_ENGINE_TO | DASH_WARN_BODY_TO)))
    rgbLedWrite(RGB_BUILTIN, 20, 0, 0);  // red
  else if (warnings)
    rgbLedWrite(RGB_BUILTIN, 15, 10, 0);  // yellow
  else
    rgbLedWrite(RGB_BUILTIN, 0, 15, 0);  // green
}

static void printCluster() {
  twai_status_info_t s;
  twai_get_status_info(&s);
  if (timedOut(engTrack, CAN_PERIOD_ENGINE_MS)) {
    Serial.printf("[DASH] -- no engine data --            | TEC=%lu REC=%lu",
                  (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter);
  } else {
    Serial.printf("[DASH] %3u km/h %4u rpm G%u %3d C thr=%3u%% fuel=%3u%% %2u.%02uV | TEC=%lu REC=%lu",
                  engine.speed_kmh, engine.rpm, engine.gear, engine.coolant_c, engine.throttle_pct,
                  body.fuel_pct, body.battery_mv / 1000, (body.battery_mv % 1000) / 10,
                  (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter);
  }
  Serial.print(" |");
  if (!warnings) Serial.print(" OK");
  if (warnings & DASH_WARN_OVERHEAT) Serial.print(" OVERHEAT");
  if (warnings & DASH_WARN_OVERSPEED) Serial.print(" OVERSPEED");
  if (warnings & DASH_WARN_LOW_FUEL) Serial.print(" LOW_FUEL");
  if (warnings & DASH_WARN_LOW_BATT) Serial.print(" LOW_BATT");
  if (warnings & DASH_WARN_ENGINE_TO) Serial.print(" ENGINE_TIMEOUT");
  if (warnings & DASH_WARN_BODY_TO) Serial.print(" BODY_TIMEOUT");
  if (warnings & DASH_WARN_E2E_ERROR) Serial.print(" E2E_ERROR");
  if (warnings & DASH_WARN_CHECK_ENG) Serial.print(" CHECK_ENGINE");
  Serial.println();
}

// ============================================================ main
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CAN Vehicle Monitor - Dashboard node (ESP32-S3) ===");
  Serial.println("Type 'l' + Enter to toggle frame logging.");
  canStart();
}

void loop() {
  static uint32_t lastTx = 0, lastPrint = 0, lastE2eReset = 0;

  // 1) Drain every received frame (non-blocking).
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) handleFrame(m);

  // 2) Bus errors / bus-off.
  handleAlerts();

  // 3) Periodic jobs.
  uint32_t now = millis();
  if (now - lastTx >= CAN_PERIOD_DASH_MS) {
    lastTx = now;
    warnings = computeWarnings();
    sendDashStatus();
    setLed();
  }
  if (now - lastE2eReset >= 3000) {  // E2E warning stays on for up to 3 s
    lastE2eReset = now;
    e2eErrorsRecent = 0;
  }
  if (!logFrames && now - lastPrint >= 1000) {
    lastPrint = now;
    printCluster();
  }

  // 4) Tiny serial command: 'l' toggles logging.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'l') {
      logFrames = !logFrames;
      Serial.printf("frame logging %s\n", logFrames ? "on" : "off");
    }
  }
}
