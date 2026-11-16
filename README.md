# CAN Bus Vehicle Monitoring System

[![firmware-build](https://github.com/GITHUB_USER/can-vehicle-monitor/actions/workflows/build.yml/badge.svg)](https://github.com/GITHUB_USER/can-vehicle-monitor/actions)

A two-node **CAN 2.0A** network at **125 kbit/s** that exchanges simulated
vehicle data, with **interrupt-driven firmware**, a **node state machine**,
**error and bus-off handling**, **end-to-end (E2E) message protection** and a
**UART diagnostic console**.

| Node | Hardware | CAN controller | Role |
|---|---|---|---|
| Engine ECU | STM32 NUCLEO-F446RE | bxCAN (STM32 HAL) | simulates the vehicle, sends ENGINE/BODY status, supervises the dashboard |
| Dashboard | ESP32-S3-DevKitC-1 | TWAI (ESP-IDF driver, Arduino) | validates frames, raises warnings, sends DASH_STATUS |

No transceiver chips were available, so the nodes share a **single-wire
diode bus** (wired-AND with a pull-up), see **[docs/wiring.md](docs/wiring.md)**.

## Message database

Defined once in [`firmware/common/can_messages.h`](firmware/common/can_messages.h),
used by both nodes, and published as [`docs/vehicle.dbc`](docs/vehicle.dbc)
for tools like SavvyCAN or `cantools`.

| ID | Name | Sender | Period | Content |
|---|---|---|---|---|
| 0x0C0 | ENGINE_STATUS | ECU | 100 ms | rpm, speed, coolant, throttle, gear, check-engine |
| 0x2A0 | DASH_STATUS | Dashboard | 200 ms | warning flags, RX statistics |
| 0x3C0 | BODY_STATUS | ECU | 500 ms | battery, fuel, ignition, door, seat belt |

Every frame carries a **4-bit rolling counter** and a **checksum that
includes the CAN ID**, so receivers detect corrupted, lost, repeated or
misrouted frames. Missing messages for 5 periods are reported as **timeouts**.

## Engine ECU state machine

```
 INIT ──> DEGRADED <──────────────> RUNNING
             │   dashboard silent /    │
             │   dashboard back        │
             └──────> BUS_OFF <────────┘   (TEC > 255)
                        │ wait 1 s
                        v
                    RECOVERING ──> RUNNING / DEGRADED
```

Green LED: 1 Hz = RUNNING, 5 Hz = DEGRADED, solid = BUS_OFF.
ESP32-S3 RGB LED: green = OK, yellow = warning, red = timeout or bus error.

## Things to try (UART console on the STM32, 115200 baud)

| Command | What you will see |
|---|---|
| `drive accel` (or press B1) | speed and rpm rise; above 130 km/h the dashboard raises OVERSPEED |
| `fuel 5` | dashboard raises LOW_FUEL |
| `fault crc` | 10 corrupted frames → dashboard rejects them, E2E_ERROR warning |
| `fault counter` | frozen counter → dashboard reports counter jumps |
| `mute on` | ECU stops sending → dashboard: ENGINE_TIMEOUT, red LED |
| unplug the ESP32 | no ACK → TEC rises, ECU goes error-passive and DEGRADED |
| `log on` / `status` | every frame in hex / error counters, TEC, REC, last error code |

## Build & test

* STM32: [docs/cubemx-config.md](docs/cubemx-config.md), then build in STM32CubeIDE.
* ESP32-S3: open `firmware/esp32s3_dashboard_node` in the Arduino IDE (board *ESP32S3 Dev Module*).
* Host unit tests for the message layer: `make -C tests`
* CI compiles the STM32 firmware, runs the unit tests and checks both nodes use an identical message database.

## Logic analyser captures

<!-- Add your PulseView screenshots: a decoded 0x0C0 frame and the ACK slot -->

## What I learned
<!-- 3-5 bullets in your own words -->
