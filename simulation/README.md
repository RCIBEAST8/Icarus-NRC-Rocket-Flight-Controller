# Simulation

`NRC_Rocket.ork` is the OpenRocket model, `AeroTech_G78G.eng` is the motor curve,
and `ICARUS_Flight_Data_Logger_v2.m` is the MATLAB script I use to read flight
and sim logs back.

## What OpenRocket predicts

The model holds several saved runs. They land between 562.7 m and 613.7 m
apogee, with max velocity between 121.6 and 124.5 m/s.

Those are clean numbers, brakes stowed the whole way up. The flight code targets
560 m, which sits under every run here, so the controller always has something to
trim off. The slowest run only clears the target by about 3 m though, so that's
the case worth re-running when the model gets updated.

## Motor, AeroTech G78G-7

| | |
|---|---|
| Total impulse | 109.9 Ns |
| Peak / average thrust | 101.9 N / 78 N |
| Burn time | 1.4 s |
| Propellant / total mass | 59.7 g / 125.0 g |
| Size | 29 x 146 mm |
| Delay | 7 s |

Don't swap in the ThrustCurve "G78G/L" RASP file. It's a different variant
(133.2 Ns, 1.7 s burn, 128 mm long) and it does not match this motor. It'll run
fine and quietly give you the wrong trajectory, which is worse than it failing.
The curve in here is modelled off published AeroTech / Apogee data and hits total
impulse, peak and mean to inside 0.1%.

The 7 second ejection delay doesn't get used. We take the ejection charge out
before flight and recovery is triggered by a separate EggTimer Apogee altimeter,
so the flight code ignores motor driven deployment completely.

## Reading logs back

`ICARUS_Flight_Data_Logger_v2.m` pulls the SD log in and plots altitude,
velocity, servo angle, the Cd the controller asked for and what apogee it was
predicting at the time. v2 works out the column count from the header instead of
hardcoding it, so it takes both the 18 column real flight log and the 20 column
sim log (the sim one carries `TrueAlt` and `TrueVel` ground truth out of
`Xiao_sim_test`).

Set `CFG.file` to point it at something, or just drop it in a folder full of CSVs
and it grabs the newest one.

Check `CFG.targetApogee` before you read anything into a plot. It defaults to
530 m and the flight firmware now targets 560 m, so if they don't match the error
traces will look wrong.

One that's easy to get backwards: 126 degrees is stowed (minimum drag) and 0 is
fully open (maximum drag).
