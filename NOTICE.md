# Attribution

Icarus is the Coventry University entry to the UKSEDS National Rocketry
Championship. It's the work of about ten people. This repo isn't the whole
vehicle and it doesn't speak for the team.

## My part

I was avionics lead. What's in this repo and what I did on it:

- The avionics PCB (shown as renders) and all the flight firmware in `firmware/`.
- The CFD in `cfd/`, including case setup, meshing and the clean and deployed
  drag coefficients the controller runs on.
- Airframe manufacture, plus a share of the wider vehicle design.

Everything else about Icarus, including structures, recovery and payload
integration, was other people on the team and isn't documented here.

## What I've deliberately left out

The customer payload spec. It was developed under the competition brief and it
isn't mine to publish, so no part of it is in this repo or its history.

The PCB design source. Schematic, layout, netlist and fab outputs are all
withheld and blocked in `.gitignore`. The board is shown as renders.

## Other people's stuff

The AeroTech G78G-7 curve in `simulation/` is modelled from published
manufacturer data (AeroTech / Apogee Components) and is in here so the flight sim
can be reproduced. OpenRocket, OpenFOAM, KiCad and the Arduino/ESP32 toolchain
belong to their respective projects.

## Licence

There isn't one. No licence file means all rights reserved by default, which is
where I want it until the team agrees on something. Ask me before reusing.
