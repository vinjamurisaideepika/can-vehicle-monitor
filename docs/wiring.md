# Wiring: a CAN bus without transceivers

A real CAN bus uses transceiver chips (MCP2551, TJA1050, SN65HVD230) that
drive two differential wires, CANH and CANL. We don't have any, but for short
distances on one breadboard CAN controllers can share **one wire** using
**diodes** (a "wired-AND" bus). This is a well-known lab technique.

## Why it works

CAN has two bus levels:

* **recessive (1):** nobody drives the bus, so the pull-up holds it HIGH
* **dominant (0):** at least one node pulls it LOW

Each controller's TX pin connects to the bus through a diode. When TX goes
LOW, the diode conducts and pulls the bus LOW (dominant). When TX is HIGH,
the diode blocks, so a HIGH node can never fight a LOW node. Every RX pin
reads the shared bus line, so each node sees its own bits (CAN needs that
for arbitration) and the other node's ACK bit.

```
                         3V3 (Nucleo CN6)
                           │
                         [4.7k]  pull-up = recessive level
                           │
   BUS LINE ───────────────┼───────────────────────┬───────────────────┐
     │          │                                  │                   │
     │       ▲ anode                            ▲ anode                │
     │      1N4148                             1N4148                  │
     │       │ cathode (band)                     │ cathode (band)     │
     │       │                                    │                    │
  PA11      PA12                               GPIO4                 GPIO5
  CAN1_RX   CAN1_TX                            TWAI_TX               TWAI_RX
  └──── NUCLEO-F446RE ───┘                     └──── ESP32-S3 ────────┘

   GND (Nucleo) ─────────────────────────────── GND (ESP32-S3)   ← mandatory
```

**Diode direction:** the **band (cathode) goes to the TX pin**, the other leg
(anode) goes to the bus line.

## Pin map

| Signal | NUCLEO-F446RE | Where | ESP32-S3 |
|---|---|---|---|
| CAN TX → diode cathode | PA12 (CAN1_TX) | Morpho **CN10 pin 12** | GPIO4 |
| CAN RX ← bus line | PA11 (CAN1_RX) | Morpho **CN10 pin 14** | GPIO5 |
| Pull-up 4.7 kΩ to 3V3 | 3V3 | CN6 | - |
| GND | GND | CN6 | GND |

## Which diodes?

* **1N4148** (small glass diode, black band): ideal, switches in nanoseconds.
* **Schottky** (BAT85, 1N5817): even better, lower voltage drop.
* **1N4007 / 1N400x** (black plastic, rectifier): slow to switch off (~µs).
  It may still work, but if you see errors, lower the bit rate to
  **50 kbit/s** on both nodes (STM32 prescaler 60; ESP32 `TWAI_TIMING_CONFIG_50KBITS()`).

The LOW level on the bus is the diode drop (~0.6 V). That is below what both
chips read as LOW (STM32 ≈ 1.0 V, ESP32-S3 ≈ 0.8 V), but not by much: keep
wires short, and use a Schottky diode if you have one.

## Logic analyser

| Channel | Signal | PulseView decoder |
|---|---|---|
| D0 | Bus line | **CAN**, bit rate 125000 |
| D1 | STM32 TX (PA12) | shows which node drives which bit |
| D2 | ESP32 TX (GPIO4) | |
| GND | GND | |

Sample rate **≥ 2 MHz**. In the decoded frame look for: the ID (0x0C0, 0x3C0,
0x2A0), DLC 8, the 8 data bytes, the CRC and the **ACK slot**, which the
*other* node pulls dominant (you can see it on the other node's TX channel).
