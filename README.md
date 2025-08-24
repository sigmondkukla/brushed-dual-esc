# brushed-dual-esc

Dual 3A ESC for 2S-3S 1lb combat robots. Designed for Clarkson Combat Robotics for use in freshman competition.

Implements an STM32C011 MCU to generate PWM for motor drivers from RX channel inputs.
All high-current inputs and outputs on the board use XT30PW wire-to-board connectors.
Two RX channels may be used as inputs, and a 5V 1A (linear) BEC is available on the RX channel headers.

## Screenshots
![3D perspective image of PCB](image/3d-perspective.png)
![3D top view image of PCB](image/3d-top.png)
![Schematic diagram](image/schematic.png)
![PCB top layer](image/pcb.png)
![STM32CubeMX configuration](image/cubemx.png)