# Project guidelines for Claude

## Language

- All code comments must be written in **English**, without exception.
- Conversation with the user is in French (user preference).

## Architecture

- Bare-metal STM32H7 flight controller. No RTOS.
- Scheduler: task queue with fixed frequency slots (4 kHz, 2 kHz, 1 kHz, 500 Hz, 100 Hz).
- All logging must go through `debugPrintLoop` (100 Hz). Never log directly from `ahrsLoop` or any other high-frequency task.

## AHRS / Madgwick

- The original `MadgwickFilter::compute()` must not be modified. Only `computeMARG()` may be added or changed.
- Earth frame convention: **NWU (North-West-Up)**. Earth X = magnetic North, Earth Y = West, Earth Z = Up.

## ESP32 firmware

- Located at `C:\Users\louis\Documents\Repos\Louis-P35\Ground_Control_Station\esp32\firmware\microflight_esp32_firmware`. **Read-only — never modify.**

## Answers

- Do not give off-the-cuff answers. Either say "I don't know" or reason carefully and search before answering.
