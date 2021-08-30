# Marlin2ForPipetBot 3D Printer and Lab Robot CNC Firmware
 
Additional documentation can be found in the 
repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot), the [Wiki](https://github.com/DerAndere1/Marlin/wiki)
or on the [PipetBot-A8 project homepage](https://derandere.gitlab.io/pipetbot-a8) 
that is part of the [authors homepage](https://derandere.gitlab.io). 
For CNC machines with additional axes I, (J, (K)) that drive pumps or other tools,
e.g. lab robots (liquid handling robots, "pipetting robots"). 
Please test this firmware and let us know if it misbehaves in any way. 
Volunteers are standing by!

## Marlin2ForPipetBot Branch

__Not for production use. Use with caution!__

Marlin2forPipetBot is a branch of the Marlin fork by DerAndere (based on 
https://github.com/MarlinFirmware/Marlin/commit/90cd1ca68d3f4f5ede56cbea4913f06ca4782a94 ). 

This branch is for patches to the latest Marlin2ForPipetBot release version.

Marlin2ForPipetBot supports up to six non-extruder axes (LINEAR_AXES) plus 
extruders (e.g. XYZIJK+E or XYZUVW+E). 

Default axis names are:

| LINEAR_AXES | Axis codes        |
|------------|-------------------|
|           3|X, Y, Z, E         |
|           4|X, Y, Z, A, E      |
|           5|X, Y, Z, A, B, E   |
|           6|X, Y, Z, A, B, C, E|

Example syntax for movement (G-code G1) with LINEAR_AXES 6: 
```
G1 [Xx.xxxx] [Yy.yyyy] [Zz.zzzz] [Aa.aaaa] Bb.bbbb] [Cc.cccc] [Ee.eeee] [Ff.ffff]
```

## Configuration

Configuration is done by editing the file Marlin/Configuration.h. E.g. change

`define LINEAR_AXES 3`

to: 

`define LINEAR_AXES 4`

Important options are:

### `FOAMCUTTER_XYUV`

Define `FOAMCUTTER_XYUV` kinematics for a hot wire cutter with parallel horizontal axes X, U where the hights
of the two wire ends are controlled by parallel axes Y, V.

### `LINEAR_AXES`

`LINEAR_AXES`: 
The number of axes that are not used for extruders (axes that
benefit from endstops and homing). `LINEAR_AXES` > `3` requires definition of
`[[I, [J, [K]]]_STEP_PIN`, `[I, [J, [K]]]_ENABLE_PIN`, `[I, [J, [K]]]_DIR_PIN`,
`[I, [J, [K]]]_STOP_PIN`, `USE_[I, [J, [K]]][MIN || MAX]_PLUG` and definition of the
respective values of `DEFAULT_AXIS_STEPS_PER_UNIT`, `DEFAULT_MAX_FEEDRATE`,
`DEFAULT_MAX_ACCELERATION`, `AXIS_RELATIVE_MODES`, `MICROSTEP_MODES` and
`MANUAL_FEEDRATE`.

Allowed values: [3, 4, 5, 6]


### `AXIS4_NAME`

`AXIS4_NAME`, `AXIS5_NAME`, `AXIS6_NAME`:
Axis codes for additional axes:
This defines the axis code that is used in G-code commands to 
reference a specific axis. 

   * 'A' for rotational axis parallel to X
   * 'B' for rotational axis parallel to Y
   * 'C' for rotational axis parallel to Z
   * 'U' for secondary linear axis parallel to X
   * 'V' for secondary linear axis parallel to Y
   * 'W' for secondary linear axis parallel to Z

Regardless of the settings, firmware-internal axis names are
I (AXIS4), J (AXIS5), K (AXIS6).

Allowed values: ['I', 'J', 'K', 'A', 'B', 'C', 'U', 'V', 'W'] 

## Building Marlin2ForPipetBot

To build Marlin2ForPipetBot you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The MarlinFirmware team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html).

The different branches in the git repository https://github.com/DerAndere1/Marlin reflect different stages of development: 
 
- [9axis_PR1](https://github.com/DerAndere1/Marlin/tree/6axis_PR1) branch: A long-time goal is to bring support for up to 10 non-extruder axes into upstream MarlinFirmware/Marlin.

- [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) branch is more stable but outdated. This branch is the release branch for [tagged releases of Marlin2ForPipetBot firmware](https://github.com/DerAndere1/Marlin/tags). It is based on Marlin bugfix-2.0.x from August 2020 (commit https://github.com/DerAndere1/Marlin/commit/638f6f0f0607399bce82123663f5463380f83ce4) 

- [Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev) is used to develop and test bugfixes for Marlin2ForPipetBot. After successful testing, it will be merged into Marlin2ForPipetBot. 

- Other branches: Deprecated legacy code.

## Hardware Abstraction Layer (HAL)

Marlin 2.0 introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility. Retaining AVR compatibility and a single code-base is important to us, because we want to make sure that features and patches get as much testing and attention as possible, and that all platforms always benefit from the latest improvements.

### Current HALs

  #### AVR (8-bit)

  board|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino AVR](https://www.arduino.cc/)|ATmega, ATTiny, etc.|16-20MHz|64-256k|2-16k|5V|no

  #### DUE

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue), [RAMPS-FD](https://www.reprap.org/wiki/RAMPS-FD), etc.|[SAM3X8E ARM-Cortex M3](https://www.microchip.com/wwwproducts/en/ATsam3x8e)|84MHz|512k|64+32k|3.3V|no

  #### ESP32

  board|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)|Tensilica Xtensa LX6|160-240MHz variants|---|---|3.3V|---

  #### LPC1768 / LPC1769

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Re-ARM](https://www.kickstarter.com/projects/1245051645/re-arm-for-ramps-simple-32-bit-upgrade)|[LPC1768 ARM-Cortex M3](https://www.nxp.com/products/microcontrollers-and-processors/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|100MHz|512k|32+16+16k|3.3-5V|no
  [MKS SBASE](https://reprap.org/forum/read.php?13,499322)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Selena Compact](https://github.com/Ales2-k/Selena)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Azteeg X5 GT](https://www.panucatt.com/azteeg_X5_GT_reprap_3d_printer_controller_p/ax5gt.htm)|LPC1769 ARM-Cortex M3|120MHz|512k|32+16+16k|3.3-5V|no
  [Smoothieboard](https://reprap.org/wiki/Smoothieboard)|LPC1769 ARM-Cortex M3|120MHz|512k|64k|3.3-5V|no

  #### SAMD51

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Adafruit Grand Central M4](https://www.adafruit.com/product/4064)|[SAMD51P20A ARM-Cortex M4](https://www.microchip.com/wwwproducts/en/ATSAMD51P20A)|120MHz|1M|256k|3.3V|yes

  #### STM32F1

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32)|[STM32F1](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html) ARM-Cortex M3|72MHz|256-512k|48-64k|3.3V|no
  [Geeetech3D GTM32](https://github.com/Geeetech3D/Diagram/blob/master/Rostock301/Hardware_GTM32_PRO_VB.pdf)|[STM32F1](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html) ARM-Cortex M3|72MHz|256-512k|48-64k|3.3V|no

  #### STM32F4

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [STEVAL-3DP001V1](https://www.st.com/en/evaluation-tools/steval-3dp001v1.html)|[STM32F401VE Arm-Cortex M4](https://www.st.com/en/microcontrollers-microprocessors/stm32f401ve.html)|84MHz|512k|64+32k|3.3-5V|yes

  #### Teensy++ 2.0

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy++ 2.0](https://www.microchip.com/wwwproducts/en/AT90USB1286)|[AT90USB1286](https://www.microchip.com/wwwproducts/en/AT90USB1286)|16MHz|128k|8k|5V|no

  #### Teensy 3.1 / 3.2

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)|[MK20DX256VLH7](https://www.mouser.com/ProductDetail/NXP-Freescale/MK20DX256VLH7) ARM-Cortex M4|72MHz|256k|32k|3.3V-5V|yes

  #### Teensy 3.5 / 3.6

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|[MK64FX512VMD12](https://www.mouser.com/ProductDetail/NXP-Freescale/MK64FX512VMD12) ARM-Cortex M4|120MHz|512k|192k|3.3-5V|yes
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|[MK66FX1M0VMD18](https://www.mouser.com/ProductDetail/NXP-Freescale/MK66FX1M0VMD18) ARM-Cortex M4|180MHz|1M|256k|3.3V|yes

  #### Teensy 4.0 / 4.1

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)|[IMXRT1062DVL6A](https://www.mouser.com/new/nxp-semiconductors/nxp-imx-rt1060-crossover-processor/) ARM-Cortex M7|600MHz|1M|2M|3.3V|yes
  [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)|[IMXRT1062DVJ6A](https://www.mouser.com/new/nxp-semiconductors/nxp-imx-rt1060-crossover-processor/) ARM-Cortex M7|600MHz|1M|2M|3.3V|yes

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the ([bugfix-2.0.x](https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.0.x)) branch.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin 2.0.x life-cycle.
- Follow the [Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit Feature Requests and Bug Reports to the [Issue Queue](https://github.com/MarlinFirmware/Marlin/issues/new/choose). Support resources are also listed there.
- Whenever you add new features, be sure to add tests to `buildroot/tests` and then run your tests locally, if possible.
  - It's optional: Running all the tests on Windows might take a long time, and they will run anyway on GitHub.
  - If you're running the tests on Linux (or on WSL with the code on a Linux volume) the speed is much faster.
  - You can use `make tests-all-local` or `make tests-single-local TEST_TARGET=...`.
  - If you prefer Docker you can use `make tests-all-local-docker` or `make tests-all-local-docker TEST_TARGET=...`.

### [RepRap.org Wiki Page](https://reprap.org/wiki/Marlin)

## Credits

The current Marlin dev team consists of:

 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA &nbsp; [![Flattr Scott](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=thinkyhead&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)
 - Roxanne Neufeld [[@Roxy-3D](https://github.com/Roxy-3D)] - USA
 - Bob Kuhn [[@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)] - USA
 - Chris Pepper [[@p3p](https://github.com/p3p)] - UK
 - João Brazio [[@jbrazio](https://github.com/jbrazio)] - Portugal
 - Erik van der Zalm [[@ErikZalm](https://github.com/ErikZalm)] - Netherlands &nbsp; [![Flattr Erik](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

Marlin2ForPipetBot is modified by:

 - DerAndere [[@DerAndere1](https://github.com/DerAndere1)] - Germany
 - Garbriel Beraldo [@GabrielBeraldo](https://github.com/GabrielBeraldo)] - Brasil
 - Olivier Briand [@hobiseven](https://github.com/hobiseven)] - France
 - Wolverine [@MohammadSDGHN](https://github.com/MohammadSDGHN) - Undisclosed 
 - bilsef [@bilsef](https://github.com/bilsef) - Undisclosed 
 - FNeo31 [@FNeo31](https://github.com/FNeo31) - Undisclosed 

## License

Marlin2ForPipetBot is published under the [GPL license](https://github.com/DerAndere1/Marlin/blob/Marlin2ForPipetBot/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
