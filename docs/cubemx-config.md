# STM32CubeMX configuration - Engine ECU node

New project: **Board Selector → NUCLEO-F446RE**, name `stm32_engine_ecu`,
location `firmware/`, answer **Yes** to default peripheral initialisation.

## Clock Configuration
* HCLK = **84** MHz → APB1 = 42 MHz (the CAN clock), APB1 timers = 84 MHz.

## Pinout & Configuration

| Peripheral | Setting | Value |
|---|---|---|
| **CAN1** | Activated | tick |
| | Pins | **PA11 → CAN1_RX**, **PA12 → CAN1_TX** (click in chip view if CubeMX chose PB8/PB9) |
| | Prescaler (for Time Quantum) | **24** → tq = 571.4 ns |
| | Time Quanta in Bit Segment 1 | **10 Times** |
| | Time Quanta in Bit Segment 2 | **3 Times** |
| | ReSynchronization Jump Width | **2 Times** |
| | → CubeMX shows **Baud Rate = 125000 bit/s** | check this! |
| | Operating Mode | **Loopback** for lesson 1, then **Normal** |
| | Automatic Bus-Off Management | **Disable** (our state machine recovers) |
| | Automatic Retransmission | **Enable** |
| | NVIC | tick **CAN1 TX**, **CAN1 RX0**, **CAN1 SCE** interrupts |
| **TIM6** | Prescaler / Period | **8399** / **99** → 100 Hz; NVIC: TIM6 global interrupt |
| **USART2** | 115200 8N1 | NVIC: USART2 global interrupt |
| **NVIC** | EXTI line[15:10] | tick (button B1) |

Bit timing explained: 42 MHz / 24 = 1.75 MHz → one tq = 571 ns.
One bit = 1 (sync) + 10 + 3 = 14 tq = 8 µs → **125 kbit/s**, sampled at
11/14 = **78.6 %** of the bit (the ESP32 TWAI samples at 80 %).

## Add our code
1. Make sure `Core/Src/app/` and `Core/Inc/app/` from this repo are in the project.
2. Add the shared message database to the include path:
   **Project → Properties → C/C++ Build → Settings → MCU GCC Compiler →
   Include paths → Add →** `${ProjDirPath}/../common` (that is `firmware/common`).
3. In `main.c`: `#include "app/app.h"` in *USER CODE Includes*, `app_init();`
   in *USER CODE 2*, `app_loop();` in *USER CODE 3*.
