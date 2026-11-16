# CAN Bus Vehicle Monitoring System - context for Claude Code

Two CAN nodes at 125 kbit/s on a transceiver-less diode bus:
STM32 NUCLEO-F446RE "Engine ECU" (bxCAN, HAL) and ESP32-S3 "Dashboard" (TWAI, Arduino).

## Layout
- `firmware/common/can_messages.h` - THE message database (IDs, packing, checksum, counter).
  `firmware/esp32s3_dashboard_node/can_messages.h` must stay an identical copy (CI diffs them).
  `docs/vehicle.dbc` must match it too.
- `firmware/stm32_engine_ecu/` - STM32CubeIDE project; our code in `Core/{Src,Inc}/app/`.
- `tests/` - host unit tests: `make -C tests`.
- `tools/ci/` - command-line firmware build for GitHub Actions.

## Conventions
- C11, no dynamic memory, ISRs only queue/flag; work happens in `app_loop()`.
- Any change to a message layout updates the header, the sketch copy, the DBC and the tests together.

## Git rules for Claude Code
- Commit messages: Conventional Commits, `type(scope): summary` + short bullet body.
- One logical change per commit. Never commit build output, `Debug/`, `.DS_Store`,
  model weights (`*.pt`, `*.onnx`, `*.tflite`) or anything secret.
- Never rewrite history, never `git push --force`, never change git config.
- If the checks in `ship.sh` fail, stop and explain the failure instead of committing.
