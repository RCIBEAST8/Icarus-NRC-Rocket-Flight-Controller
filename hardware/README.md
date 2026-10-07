# Hardware

![Isometric](renders/pcb_iso.png)

Renders only. The schematic, layout, netlist and fabrication outputs (Gerbers,
drill files, pick and place) are deliberately not published and are blocked in
`.gitignore`. The board's here to show what I built, not for anyone to send off
to a fab. Ask me if you need more detail than this.

It's a 50.0 x 30.0 mm two layer board. The XIAO ESP32-C3 sits at `U1` and
basically everything else on there exists to power it, break its buses out, and
keep it alive while a servo shares the same supply.

| Ref | What it is |
|---|---|
| `U1` | Seeed XIAO ESP32-C3 module footprint |
| `U2` | SOT-223 linear regulator, with `C4` / `C5` either side of it |
| `C1` | bulk electrolytic, sat right next to the servo header |
| `J1`, `J2` | two I2C breakouts (SCL, SDA, 3V3, GND) for the IMU and the baro |
| `J3` | SPI header for the microSD, with CS broken out |
| `J4` | servo (GND, 5V, PWM) |
| `J7`, `J8` | XIAO headers, VBUS/3V3/GND one side, D0-D3/TX/SDA/SCL the other |
| `VIN`, `GND` | power in |
| `M2` x2 | mounting holes |

The bulk cap next to `J4` is there on purpose. A servo under stall load pulls
current in sharp bursts and without local bulk capacitance those transients drag
the rail down far enough to brown out the microcontroller mid flight. The IMU and
baro also get their own separate I2C headers rather than being daisy chained, so
each one gets a short run.

There are four renders in `renders/`: an isometric and a flat top down
(`pcb_iso.png`, `pcb_top.png`), plus the same two from underneath
(`pcb_iso_bottom.png`, `pcb_bottom.png`). The flat top down is the useful one if
you want to read the silkscreen and follow the routing.

They're generated out of the KiCad 10 project with `kicad-cli pcb render`. `U1`
and `J3` show up as plain outlines because those footprints don't have 3D models
attached, so the XIAO itself isn't shown sitting in its socket.
