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
That pad integration (`currentvelocity`) is only used to spot the launch and to
seed the in-flight estimate, it gets thrown away after that because it drifts.

In flight the accelerometer propagates velocity every 20 ms tick and the
barometer corrects it, pulling the estimate 35% of the way toward the alpha-beta
baro velocity each 10 Hz update. During the burn that correction is switched off
completely, so for the first 1.4 s it's running on the accelerometer alone.

Apogee detect needs peak altitude above 50 m first, then fires on fused velocity
< -2.0 m/s or 5 falling samples in a row. Landing detect wants to be below 25 m
AGL with altitude stable within 3 m and acceleration within 1.5 m/s^2 of 1 g,
held for 5 seconds.

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
| `apogeeTgt` | 560 m | target apogee (about 1840 ft) |
| `overshootPenalty` | 1.25 | makes the controller prefer undershooting |
| `MPC_CANDIDATES` | 41 | how many drag values it tries per cycle |

`padMass` is the wet mass at 0.789 kg, `dryMass` is 0.7293 kg. Through the burn
the mass is interpolated linearly between the two on elapsed time, then held at
dry mass after burnout. A straight line isn't the real burn profile, but the burn
is 1.4 s and the MPC doesn't arm until 0.1 s after it, so in practice the
controller only ever sees dry mass anyway.

## Real-time classification

Soft real-time. It's a polled periodic trigger in `loop()`, not a timer ISR and
not a released RTOS task. No measured worst case execution time, no deadline miss
detection. SD writes are queued so they can't stall the loop.

Guards that are in there: density clamped to 0.6-1.4 kg/m^3, velocity clamped to
-150 to 200 m/s, barometric altitude rejected outside -500 to 10000 m, a divide
by zero guard on the drag term, servo rate limiting and a 5 degree deadband.

## Known issues

`apogeeTgt` is 560 m here, but `simulation/ICARUS_Flight_Data_Logger_v2.m`
defaults to 530 m. Set the analyser to match whichever build produced the log or
the error traces will lie to you.

The OpenRocket runs in `simulation/` land between 562.7 and 613.7 m clean, so a
560 m target sits under all of them and the brakes always have something to shed.
The margin at the bottom of that spread is only about 3 m though, so on a slow
day there's very little for the controller to work with.

A couple of comments are out of date. There's still an `airbreak` spelling on
lines 50 and 463, and a comment on line 473 that says 11 sims when the sweep is
41.
