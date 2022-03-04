# Marlin2ForPipetBot 3D Printer and Lab Robot CNC Firmware
 
Additional documentation can be found in the 
repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot), the [Wiki](https://github.com/DerAndere1/Marlin/wiki)
or on the [PipetBot-A8 project homepage](https://derandere.gitlab.io/pipetbot-a8) 
that is part of the [authors homepage](https://derandere.gitlab.io). 
For CNC machines with additional axes (I, J, K, U, V, W) that can be used for indexed machining or to drive pumps or other tools,
e.g. lab robots (liquid handling robots, "pipetting robots"). 
Please test this firmware and let us know if it misbehaves in any way. 
Volunteers are standing by!

## Marlin2ForPipetBot Branch

__Not for production use. Use with caution!__

Marlin2forPipetBot is a branch of the Marlin fork by DerAndere (based on 
https://github.com/MarlinFirmware/Marlin/commit/d6b332f4c7a79553a41c4a6166f21f802be930c8). 

This branch is for patches to the latest Marlin2ForPipetBot release version.

Marlin2ForPipetBot supports up to nine non-extruder axes (NUM_AXES) plus 
extruders (e.g. XYZABCUVW+E or XYZABCW+E or XYZCUVW+E or XYZABC+E or XYZUVW+E). 

Default axis names are:

| NUM_AXES | Axis codes                 |
|----------|----------------------------|
|         3|X, Y, Z, E                  |
|         4|X, Y, Z, A, E               |
|         5|X, Y, Z, A, B, E            |
|         6|X, Y, Z, A, B, C, E         |
|         7|X, Y, Z, A, B, C, U, E      |
|         8|X, Y, Z, A, B, C, U, V, E   |
|         9|X, Y, Z, A, B, C, U, V, W, E|

Example syntax for movement (G-code G1) with NUM_AXES 9: 
```
G1 [Xx.xxxx] [Yy.yyyy] [Zz.zzzz] [Aa.aaaa] Bb.bbbb] [Cc.cccc] [Uu.uuuu] [Vv.vvvv] [Ww.wwww] [Ee.eeee] [Ff.ffff]
```
Parameters:

`X`, `Y`, `Z`: position in the cartesian coordinate system consisting of primary linear axes X, Y and Z. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

`A`, `B`, `C`: angular position in the pseudo-cartesian coordinate system consisting of rotational axes A, B, and C that are parallel (wrapped around) axes axes X, Y and Z. Unit: degrees

`U`, `V`, `W`: position in the cartesian coordinate system consisting of secondary linear axes U, V and W that are parallel to axes X, Y and Z. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

`E`: distance the E stepper should move. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

`F`: Feedrate as defined by LinuxCNC (extension of NIST RS274NGC interpreter - version 3):

- For motion involving one or more of the X, Y, and Z axes (with or without motion of other axes), the feed rate means length units per minute along the
programmed XYZ path, as if the other axes were not moving.
- For motion of one or more of the secondary linear axes (axis names 'U', 'V', or 'W') with the X, Y , and Z axes not moving (with or without motion of rotational axes), the feed rate means length units per minute along the
programmed UVW path (using the usual Euclidean metric in the UVW coordinate system), as if the rotational axes were not moving.
- For motion of one or more of the rotational axes (axis names 'A', 'B' or 'C') with linear axes not moving, the rate is
applied as follows. Let dA, dB, and dC be the angles in degrees through which the A, B,
and C axes, respectively, must move. Let D = sqrt((dA)^2 + (dB)^2 + (dC)^2). Conceptually, D is a
measure of total angular motion, using the usual Euclidean metric. Let T be the amount
of time required to move through D degrees at the current feed rate in degrees per
minute. The rotational axes should be moved in coordinated linear motion so that the
elapsed time from the start to the end of the motion is T plus any time required for
acceleration or deceleration.

To change the feed rate interpretation the option `ARTICULATED_ROBOT_ARM` can be defined. With that option enabled, feed reference are all axes. This means that in all cases all axes are moved in coordinated linear motion so that the time (in minutes) required for the move is T = sqrt((dA)^2 + (dB)^2 + (dC)^2 + (dU)^2 + (dV)^2 + (dW)^2) / F plus any time for
acceleration or deceleration.

## Configuration

Configuration is done by editing the file Marlin/Configuration.h. E.g. change

`define NUM_AXES 3`

to: 

`define NUM_AXES 4`

Important options are:

### `FOAMCUTTER_XYUV`

Define `FOAMCUTTER_XYUV` kinematics for a hot wire cutter with parallel horizontal axes X, U where the hights
of the two wire ends are controlled by parallel axes Y, V. Currently only works with LINEAR_AXES 5. A dummy pin number can be assigned to pins for the unused Z axis. Leave FOAMCUTTER_XYUV disabled for default behaviour (stepper velocities are calculated using multidimensional linear interpolation over all axes). Host software and CAM software for 4 axis foam cutters can be found at https://rckeith.co.uk/file-downloads/ and https://www.jedicut.com/en/download/ .

### `NUM_AXES`

`NUM_AXES`: 
The number of axes that are not used for extruders (axes that
benefit from endstops and homing). `NUM_AXES` > `3` requires definition of
`[[I, [J, [K...]]]_STEP_PIN`, `[I, [J, [K...]]]_ENABLE_PIN`, `[I, [J, [K...]]]_DIR_PIN`,
`[I, [J, [K...]]]_STOP_PIN`, `USE_[I, [J, [K...]]][MIN || MAX]_PLUG`, 
`[I, [J, [K...]]]_ENABLE_ON`, `DISABLE_[I, [J, [K...]]]`, `[I, [J, [K...]]]_MIN_POS`, 
`[I, [J, [K...]]]_MAX_POS`, `[I, [J, [K...]]]_HOME_DIR`, possibly `DEFAULT_[I, [J, [K...]]]JERK`, 
and definition of the respective values of `DEFAULT_AXIS_STEPS_PER_UNIT`, `DEFAULT_MAX_FEEDRATE`,
`DEFAULT_MAX_ACCELERATION`, `HOMING_FEEDRATE_MM_M`, `AXIS_RELATIVE_MODES`, `MICROSTEP_MODES`,
`MANUAL_FEEDRATE` and possibly also values of `HOMING_BUMP_DIVISOR`,  
`HOMING_BACKOFF_POST_MM`, `BACKLASH_DISTANCE_MM`.
For bed-leveling, `NOZZLE_TO_PROBE_OFFSETS` has to be extended with elemets of value 0
until the number of elements is equal to the value of `NUM_AXES`.

Allowed values: [2, 3, 4, 5, 6, 7, 8, 9]

### `AXIS4_ROTATES`

`AXIS4_ROTATES`, `AXIS5_ROTATES`, `AXIS6_ROTATES`, `AXIS7_ROTATES`, `AXIS8_ROTATES`, `AXIS9_ROTATES`:
If enabled, the corresponding axis is a rotational axis for which positions are specified in angular degrees.
For moves involving only rotational axes, feedrate is interpreted in angular degrees.
 
### `AXIS4_NAME`

`AXIS4_NAME`, `AXIS5_NAME`, `AXIS6_NAME`, `AXIS7_NAME`, `AXIS8_NAME`, `AXIS9_NAME`:
Axis codes for additional axes:
This defines the axis code that is used in G-code commands to 
reference a specific axis. Axes with name 'A', 'B' or 'C' are rotational axes for which
distances and positions must be specified in degrees. Other axes are linear axes for which
distances and positions must be specified in length units (mm in default mode (after G21) or imperial inches in inch mode (after G20))

   * 'A' for rotational axis parallel to X
   * 'B' for rotational axis parallel to Y
   * 'C' for rotational axis parallel to Z
   * 'U' for secondary linear axis parallel to X
   * 'V' for secondary linear axis parallel to Y
   * 'W' for secondary linear axis parallel to Z

Regardless of the settings, firmware-internal axis names are
I (AXIS4), J (AXIS5), K (AXIS6), U (AXIS7), V (AXIS8), W (AXIS9).

Allowed values: ['A', 'B', 'C', 'U', 'V', 'W'] 

### SAFE_BED_LEVELING_START_X

`SAFE_BED_LEVELING_START_X`, `SAFE_BED_LEVELING_START_Y`, `SAFE_BED_LEVELING_START_Z`, `SAFE_BED_LEVELING_START_I`, `SAFE_BED_LEVELING_START_J`, `SAFE_BED_LEVELING_START_K`, `SAFE_BED_LEVELING_START_U`, `SAFE_BED_LEVELING_START_V`, `SAFE_BED_LEVELING_START_W`: 
Safe bed leveling start coordinates. If enabled, the respective axis is moved to the specified position at the beginning of the bed leveling procedure.
Required e.g. with NUM_AXES >= 4, if Z probe is not perpendicular to the bed after homing.
Values must be chosen so that the bed is oriented horizontally and so that the Z-probe is oriented vertically.

### LCD_SHOW_SECONDARY_AXES

Show the position of secondary axes I[J[K]] instead of icons on an DOGM LCD (e.g. REPRAP_FULL_GRAPHICS_DISPLAY).

### QUICK_HOME_ALL_NON_Z_AXES

If all axes are homed, first raise Z, then move all axes except Z simultaneously to their home position. Once the first axis reaches its home position, the axes will be homed individually in sequence XYZIJKUVW. Requires `QUICK_HOME`.

### ARTICULATED_ROBOT_ARM

When enabled, feed rate (the F-word in G1 G-code commands) is interpreted with all axes as the feed reference. For compatibility with Marlin <= 2.0.9.3, grblHAL/grblHAL-core, Duet3D/RepRap-Firmware, synthetos/g2core and for compatibility with articulated robots (robot arms) for which inverse kinematics are not yet implemented in Marlin. For a detailed description of feedrate, see section "Marlin2ForPipetBot Branch" above.


## Building Marlin2ForPipetBot

To build Marlin2ForPipetBot you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The Marlin team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html). [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) is preconfigured for the Anet-V1.0 board of the PipetBot-A8. When using the default build environment (`default_env = melzi_optiboot`), upload of the compiled Marlin2ForPipetBot firmware to the board via USB using the optiboot bootloader requires burning of the [optiboot bootloader](https://github.com/Optiboot/optiboot) onto the board as described in the [SkyNet3D/anet-board documentation](https://github.com/SkyNet3D/anet-board).

The different branches in the git repository https://github.com/DerAndere1/Marlin reflect different stages of development: 
 
- [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) branch is the stable release branch for [tagged releases of Marlin2ForPipetBot firmware](https://github.com/DerAndere1/Marlin/tags). It is optimized and preconfigured for the [PipetBot-A8](https://derandere.gitlab.io/pipetbot-a8) by default. Currently it is based on Marlin bugfix-2.0.x from 2021-12-05, [https://github.com/MarlinFirmware/Marlin/commit/d6b332f4c7a79553a41c4a6166f21f802be930c8](https://github.com/MarlinFirmware/Marlin/commit/d6b332f4c7a79553a41c4a6166f21f802be930c8) or later. Adds support for 9 axes, including rotational axes (`AXIS*_ROTATES`)

- [Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev) branch is used to develop and test bugfixes for Marlin2ForPipetBot. After successful testing, it will be merged into Marlin2ForPipetBot.

- [6axis_PR1](https://github.com/DerAndere1/Marlin/tree/6axis_PR1) branch was merged into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/19112). This branch is now outdated. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

- [9axis_PR2](https://github.com/DerAndere1/Marlin/tree/9axis_PR2) branch is used to develop support for up to 9 non-extruder axes. A pull request targeting https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.0 was opened. This branch needs to be rebased onto https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.0.

- Other branches: Deprecated legacy code. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

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
 - HendrikJan-5D [@HendrikJan-5D](https://github.com/HendrikJan-5D) - Undisclosed

## License

Marlin2ForPipetBot is published under the [GPL license](https://github.com/DerAndere1/Marlin/blob/Marlin2ForPipetBot/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.