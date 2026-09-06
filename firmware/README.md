# Firmware

`XIAO_ESP32-C3_CODE.ino` is the flight code. `Xiao_sim_test.ino` is the same
controller wired to a synthetic flight so I can test it without burning a motor.

## Hardware it runs on

Seeed XIAO ESP32-C3 (RV32IMC). No hardware FPU, so every float operation is
emulated in software. That's the single biggest constraint on what fits in the
loop budget and it's why the apogee predictor is a closed form solution.

| Part | Bus | Notes |
|---|---|---|
| ICM-20948 | I2C | 9 axis IMU |
| BMP390 | I2C | barometer, this is the primary altitude source |
| microSD | SPI | logging, behind a FreeRTOS queue |
| airbrake servo | PWM | 0-126 deg, 126 is a mechanical limit on the linkage |

Loop runs at 20 ms (50 Hz). Barometric velocity is filtered down to 10 Hz on
purpose, slower than the loop, so the estimator isn't chasing sensor noise.

MISO isn't on the usual pin, because of boot state issues on the XIAO.

## States

`PAD_IDLE` -> `ASCENT` -> `DESCENT` -> `LANDED`

Launch detect fires on integrated IMU distance >= 1.0 m or altitude >= 50 m AGL.
The IMU integration is only used for launch detection, it never gets fused into
barometric velocity once we're flying, because it drifts badly.

Apogee detect fires on fused velocity < -2.0 m/s or 5 falling samples in a row.
Landing detect wants altitude stable within 3 m and acceleration within
1.5 m/s^2 of 1 g, held for 5 seconds.

Brakes stay stowed until burn time + 0.1 s, then the controller arms. That delay
is there to stop the servo straining against itself. Three seconds after apogee
they get driven fully open to add drag on the way down.

## Constants worth knowing

| Constant | Value | What it is |
|---|---|---|
| `alpha` / `beta` | 0.4 / 0.05 | alpha-beta filter gains on baro altitude |
| `BARO_TRUST` | 0.35 | how much of the baro/inertial gap gets corrected each tick |
| `area` | 0.0019635 m^2 | CFD reference area, has to match the CFD |
| `CDClean` / `CDMax` | 0.4724 / 0.8776 | straight out of `cfd/clean` and `cfd/deployed` |
| `apogeeTgt` | 670 m | target apogee (about 2200 ft) |
| `overshootPenalty` | 1.25 | makes the controller prefer undershooting |
| `MPC_CANDIDATES` | 41 | how many drag values it tries per cycle |

`padMass` and `dryMass` are both 0.7293 kg and they're meant to be. The mass
model deliberately holds dry mass throughout rather than interpolating down
through the burn, and the controller is gated off until after burnout anyway, so
it never sees a wet mass.

## Real-time classification

Soft real-time. It's a polled periodic trigger in `loop()`, not a timer ISR and
not a released RTOS task. No measured worst case execution time, no deadline miss
detection. SD writes are queued so they can't stall the loop.

Guards that are in there: density clamped to 0.6-1.4 kg/m^3, velocity clamped to
-150 to 200 m/s, a divide by zero guard on the drag term, servo rate limiting and
a 5 degree deadband.

## Known issues

`apogeeTgt` is 670 m here, but `simulation/ICARUS_Flight_Data_Logger_v2.m`
defaults to 530 m. Set the analyser to match whichever build produced the log or
the error traces will lie to you.

The OpenRocket model in `simulation/` tops out at 613.7 m across its saved runs,
which is below the 670 m target. The model needs re-running against the current
build before those two mean anything together.

A couple of comments are out of date. There's still an `airbreak` spelling on
line 50, and a comment on line 476 that says 11 sims when the sweep is 41.
