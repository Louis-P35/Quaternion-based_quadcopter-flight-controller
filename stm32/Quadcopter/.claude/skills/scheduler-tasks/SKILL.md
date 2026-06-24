---
description: >
  Reference for the bare-metal STM32H7 scheduler and task system. Use when adding,
  modifying, or debugging tasks, frequency slots, SPI arbitration, timing budgets,
  or task priorities. Also use when the user asks about how the scheduler works,
  which slot to put a new task in, or why a task is behaving unexpectedly.
when_to_use: >
  Trigger on: "add a task", "new task", "scheduler", "frequency slot", "SPI collision",
  "back-to-back SPI", "timing budget", "priority", "addTask", "readSpiFromESP",
  "100 Hz", "500 Hz", task timing, CPU budget, task ordering.
---

# Scheduler and Task System — STM32H7 Flight Controller

## How the Scheduler Works

**Timer 2 ISR** fires at 4 kHz and sets bits in `m_loops_frequencies_bit_fields`
(one bit per frequency slot). It runs from flash via interrupt — keep it minimal.

**`Scheduler::runTasks()`** is called in the main loop (not from the ISR). It scans
slots from index 3 (4 kHz) to 12 (1 Hz) in order. For each slot whose bit is set,
it runs all tasks in that slot sorted by priority, then clears the bit.

**Consequence**: When multiple slots fire in the same `runTasks()` call, they execute
in descending frequency order. The 500 Hz slot always runs before the 50 Hz slot.
Within a slot, lower priority number = earlier execution (0 = highest priority).

## Frequency Slots and Timing

| Slot enum       | Hz   | Period  | Index | Active |
|-----------------|------|---------|-------|--------|
| `e_4KHZ`        | 4000 | 250 µs  | 3     | ✓      |
| `e_2KHZ`        | 2000 | 500 µs  | 4     | ✓      |
| `e_1KHZ`        | 1000 | 1 ms    | 5     | ✓      |
| `e_500HZ`       |  500 | 2 ms    | 6     | ✓      |
| `e_250HZ`       |  250 | 4 ms    | 7     | ✓      |
| `e_100HZ`       |  100 | 10 ms   | 8     | ✓      |
| `e_50HZ`        |   50 | 20 ms   | 9     | ✓      |
| `e_10HZ`        |   10 | 100 ms  | 10    | ✓      |
| `e_5HZ`         |    5 | 200 ms  | 11    | ✓      |
| `e_1HZ`         |    1 | 1 s     | 12    | ✓      |
| `e_8/16/32KHZ`  | —    | —       | 0–2   | DISABLED (divider = 0) |

**Real CPU budget per slot**: The nominal period is NOT the available budget.
Higher-frequency tasks steal time from lower-frequency ones. A 100 Hz task shares
its 10 ms window with 40 × 4 kHz + 20 × 2 kHz + 10 × 1 kHz + 5 × 500 Hz
executions. Keep all tasks short. There is no preemption — a slow task delays the
next 4 kHz deadline.

## Current Task Registration (flightCore.cpp → mainSetup)

```cpp
// 4 kHz
addTask(eRead_IMU,        0,  readIMU_task,           e_4KHZ);

// 2 kHz
addTask(ePID_rate,        0,  pidRate_task,           e_2KHZ);

// 1 kHz
addTask(eAHRS,            0,  AHRS_task,              e_1KHZ);
addTask(ePID_att,         1,  pidAtt_task,            e_1KHZ);

// 500 Hz
addTask(eESCs,            0,  ESCs_task,              e_500HZ);
addTask(eRead_espSpi,     1,  readSpiFromESP_task,    e_500HZ);

// 100 Hz
addTask(eRead_opticalFlow,0,  readOpticalFlow_task,   e_100HZ);  // #if SENSOR_MTF01_ENABLED
addTask(ePID_pos,         1,  pidPos_task,            e_100HZ);

// 50 Hz
addTask(eRead_radio,      0,  readRadio_task,         e_50HZ);
addTask(eMain_fsm,        1,  mainFSM_task,           e_50HZ);

// 10 Hz
addTask(eRead_battery,    0,  readBattery_task,       e_10HZ);
```

Adding a new task requires: (1) add a `TaskType` enum value in `task.hpp`, (2) add
a C wrapper function declaration in `flightCore.hpp`, (3) implement the C wrapper
and the method in `flightCore.cpp`, (4) call `addTask(...)` in `mainSetup()`.

## SPI Arbitration — CRITICAL RULE

**All ESP32 SPI transactions are centralized in `readSpiFromESP()` at 500 Hz.**
This is non-negotiable. Do NOT add SPI calls to any other task.

### Why

`HAL_SPI_TransmitReceive` is full-duplex: every call sends MOSI and receives MISO
simultaneously. After each transaction the ESP32 needs time to rebuild its MISO
buffer. If two transactions fire within the same scheduler iteration (< 1 µs apart),
the ESP32 MISO is stale, the magic check (0xCAFE) fails, and
`parseMisoFrame()` zeroes out all sensor structs — radio, GPS, baro, MTF-01 all
read zero. This was diagnosed after multiple back-to-back collisions.

### Pending-Flag Arbitration (current implementation)

`readSpiFromESP()` runs 500 times/second. Inside it:

1. Static booleans mark each sensor as pending when its period expires.
2. **Exactly one SPI transaction** fires per call — the highest-priority pending sensor.
3. Deferred sensors keep their flag set and fire on the next available tick.

```
tick % 5  == 0  →  pendingDebug = true   (100 Hz, priority 1)
tick % 5  == 0  →  pendingMtf01 = true   (100 Hz, priority 2, always 1 tick behind debug)
tick % 50 == 0  →  pendingGps   = true   (10 Hz,  priority 3)
tick % 50 == 0  →  pendingBaro  = true   (10 Hz,  priority 4, 1 tick behind GPS)
tick % 10 == 0  →  feedSpiData() — no SPI, just copies cached MISO → radio protocol (50 Hz)
```

At tick%50==0 (four sensors all due), they fire across 4 consecutive 500 Hz ticks,
never back-to-back. At 500 Hz master rate, ~220 SPI transactions/s out of 500 slots.

### Radio Timing Guarantee

`feedSpiData()` runs inside `readSpiFromESP` (500 Hz slot, priority 1).
`radioLoop` runs in the 50 Hz slot (priority 0).
The 500 Hz slot always executes before the 50 Hz slot in a coinciding iteration.
Therefore SBUS data is always fresh when `readRadioReceiver()` is called.

**Do NOT call `feedSpiData()` from `radioLoop`.** It is already called upstream.

## Debug / Logging Rule

**Never log from `ahrsLoop`, `pidRateLoop`, `pidAttLoop`, `escLoop`, or any other
high-frequency task.** All logging goes through `debugPrintLoop()`, which is called
from `readSpiFromESP()` at 100 Hz (pendingDebug, priority 1). It cycles through 4
phases at 25 Hz each: Attitude → Status → Mag → RC.

To add a new log field: add a phase to `debugPrintLoop()` (increase phase count),
or replace an existing unused phase.

## Slot Coincidence Map

When does each slot coincide with 500 Hz (i.e., both run in the same `runTasks()` call)?

| Slot    | Coincides with 500 Hz every… |
|---------|------------------------------|
| 4 kHz   | every 4kHz tick (always)     |
| 2 kHz   | every 2nd tick               |
| 1 kHz   | every 4th tick               |
| 500 Hz  | always (it IS this slot)     |
| 100 Hz  | every 5th 500 Hz call        |
| 50 Hz   | every 10th 500 Hz call       |
| 10 Hz   | every 50th 500 Hz call       |

At a coinciding tick, slots run in descending-frequency order (500 Hz before 50 Hz,
etc.). Within each slot, tasks run in priority order (0 first).

## Anti-Patterns

- **Never add a direct SPI call outside `readSpiFromESP`.** Even one extra SPI in
  a separate task will eventually collide back-to-back with the arbiter and corrupt
  all sensor data.
- **Never log from a task faster than 100 Hz.** The UART/SPI link cannot keep up
  and it will stall the high-frequency loops.
- **Never put long-running work in the 4 kHz or 2 kHz slots.** These have ~250 µs
  and ~500 µs budgets respectively, and they share time with every lower-frequency
  slot that coincides.
- **Be careful with priority 0 at 100 Hz.** `eRead_opticalFlow` is priority 0 in
  the 100 Hz slot. Any new task that also needs priority 0 there will run before the
  optical flow sensor.

## ESP32 MISO Frame Layout (256 bytes, magic = 0xCAFE)

| Offset | Content                         |
|--------|---------------------------------|
| 0–1    | magic (0xCA, 0xFE)              |
| 2–32   | command section                 |
| 33     | has_sbus flag                   |
| 34–65  | sbus_raw[16]                    |
| 66     | has_gps flag                    |
| 67–96  | GPS payload (30 B)              |
| 97     | has_mtf01 flag                  |
| 98–106 | MTF-01 payload (9 B)            |
| 107    | has_mag flag                    |
| 108–113| GPS compass payload (6 B)       |
| 114    | has_baro flag                   |
| 115–126| Baro payload (12 B)             |

`parseMisoFrame()` is called after every SPI transaction (regardless of MOSI type)
and refreshes all sensor structs. On magic failure, all structs are zeroed.

## MOSI Frame Types

| Value | Meaning   | Sent by         |
|-------|-----------|-----------------|
| 0x01  | Attitude  | debugPrintLoop phase 0 |
| 0x02  | Status    | debugPrintLoop phase 1 |
| 0x06  | RC        | debugPrintLoop phase 3 |
| 0x07  | GPS req   | readEspGps()    |
| 0x08  | MTF-01 req| readEspMtf01()  |
| 0x09  | Mag       | debugPrintLoop phase 2 |
| 0x0A  | Baro req  | readEspBaro()   |
