# Marlin2ForPipetBot 3D Printer and Lab Robot CNC Firmware
 
Additional documentation can be found in the 
repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot), the [Wiki](https://github.com/DerAndere1/Marlin/wiki)
or on the [PipetBot-A8 project homepage](https://derandere.gitlab.io/pipetbot-a8) 
that is part of the [authors homepage](https://derandere.gitlab.io). 
For CNC machines with additional axes (I, J, K, U, V, W) that can be used for indexed machining or to drive pumps or other tools,
e.g. lab robots (liquid handling robots, "pipetting robots"). 
Please test this firmware and let us know if it misbehaves in any way. 
Volunteers are standing by!


Marlin supports up to nine non-extruder axes plus 
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

Example syntax for movement (G-code G1) with 9 axes: 
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

`//#define I_DRIVER_TYPE A4988`

to: 

`#define I_DRIVER_TYPE A4988`

Important options are:

### `X_DRIVER_TYPE`

`X_DRIVER_TYPE`, `Y_DRIVER_TYPE`, `Z_DRIVER_TYPE`, `I_DRIVER_TYPE`, `J_DRIVER_TYPE`, `K_DRIVER_TYPE`, `U_DRIVER_TYPE`, `V_DRIVER_TYPE`, `W_DRIVER_TYPE`: These settings allow Marlin to tune stepper driver timing and enable advanced options for
 stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 
 Use TMC2208/TMC2208_STANDALONE for TMC2225 drivers and TMC2209/TMC2209_STANDALONE for TMC2226 drivers.
 
 Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
          TB6560, TB6600, TMC2100,
          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE

These settings allow Marlin to tune stepper driver timing and enable advanced options for stepper drivers that support them. 
You may also override timing options in Configuration_adv.h. Each driver is associated with an axis (internal axis identifiers: 
X, Y, Z, I, J, K, U, V, W) or an extruder (E0 to E7). 
Each axis gets its own stepper control and endstops depending on the following settings:
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

Allowed values: [A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01, TB6560, TB6600, TMC2100, TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE, TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE, TMC26X, TMC26X_STANDALONE, TMC2660, TMC2660_STANDALONE, TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE]

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

### ARTICULATED_ROBOT_ARM

When enabled, feed rate (the F-word in G1 G-code commands) is interpreted with all axes as the feed reference. For compatibility with Marlin <= 2.0.9.3, grblHAL/grblHAL-core, Duet3D/RepRap-Firmware, synthetos/g2core and for compatibility with articulated robots (robot arms) for which inverse kinematics are not yet implemented in Marlin. For a detailed description of feedrate, see first section.

### `FOAMCUTTER_XYUV`

Define `FOAMCUTTER_XYUV` kinematics for a hot wire cutter with parallel horizontal axes X, U where the hights
of the two wire ends are controlled by parallel axes Y, V. Currently only works with `*_DRIVER_TYPE` defined for 5 axes (X, Y, Z, I and J). A dummy pin number can be assigned to pins for the unused Z axis. Leave `FOAMCUTTER_XYUV` disabled for default behaviour (stepper velocities are calculated using multidimensional linear interpolation over all axes). Host software and CAM software for 4 axis foam cutters can be found at https://rckeith.co.uk/file-downloads/ and https://www.jedicut.com/en/download/ .

### SAFE_BED_LEVELING_START_X

`SAFE_BED_LEVELING_START_X`, `SAFE_BED_LEVELING_START_Y`, `SAFE_BED_LEVELING_START_Z`, `SAFE_BED_LEVELING_START_I`, `SAFE_BED_LEVELING_START_J`, `SAFE_BED_LEVELING_START_K`, `SAFE_BED_LEVELING_START_U`, `SAFE_BED_LEVELING_START_V`, `SAFE_BED_LEVELING_START_W`: 
Safe bed leveling start coordinates. If enabled, the respective axis is moved to the specified position at the beginning of the bed leveling procedure.
Required for multi-axis machines (`I_DRIVER_TYPE` ... defined).
Values must be chosen so that the bed is oriented horizontally and so that the Z-probe is oriented vertically.
Note: If inverse kinematics for your machine are not implemented, bed leveling produces wrong results for all moves where the bed is not oriented horizontally or where the tool head is not oriented vertically. In these cases, bed leveling must be disabled.

## Marlin2ForPipetBot Branch

__Not for production use. Use with caution!__

Marlin2forPipetBot is a branch of the Marlin fork by DerAndere (based on 
https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789). 

This branch is for patches to the latest Marlin2ForPipetBot release version.


## Configuration

### LCD_SHOW_SECONDARY_AXES

Show the position of secondary axes I[J[K]] instead of icons on an DOGM LCD (e.g. REPRAP_FULL_GRAPHICS_DISPLAY).

### QUICK_HOME_ALL_NON_Z_AXES

If all axes are homed, first raise Z, then move all axes except Z simultaneously to their home position. Once the first axis reaches its home position, the axes will be homed individually in sequence XYZIJKUVW. Requires `QUICK_HOME`.


## Example Configurations

Before building Marlin you'll need to configure it for your specific hardware. Your vendor should have already provided source code with configurations for the installed firmware, but if you ever decide to upgrade you'll need updated configuration files. Marlin users have contributed dozens of tested example configurations to get you started. Visit the [MarlinFirmware/Configurations](https://github.com/MarlinFirmware/Configurations) repository to find the right configuration for your hardware.

## Building Marlin2ForPipetBot

To build Marlin2ForPipetBot you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The Marlin team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html). [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) is preconfigured for the Anet-V1.0 board of the PipetBot-A8. When using the default build environment (`default_env = melzi_optiboot`), upload of the compiled Marlin2ForPipetBot firmware to the board via USB using the optiboot bootloader requires burning of the [optiboot bootloader](https://github.com/Optiboot/optiboot) onto the board as described in the [SkyNet3D/anet-board documentation](https://github.com/SkyNet3D/anet-board).

The different branches in the git repository https://github.com/DerAndere1/Marlin reflect different stages of development: 
 
- [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) branch is the stable release branch for [tagged releases of Marlin2ForPipetBot firmware](https://github.com/DerAndere1/Marlin/tags). It is optimized and preconfigured for the [PipetBot-A8](https://derandere.gitlab.io/pipetbot-a8) by default. Currently it is based on MarlinFirmware/Marlin bugfix-2.1.x from 2022-06-07, [https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789](https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789) or later. In addition to MarlinFirmware/Marlin's support for 9 axes, including rotational axes (`AXISn_ROTATES`), it adds simultaneous homing of all axes except Z (`QUICK_HOME_ALL_NON_Z_AXES`), a second controller fan pin (`CONTROLLER_FAN2_PIN`), and it can be configured to show positions of secondary axes on an LCD (`LCD_SHOW_SECONDARY_AXES`).

- [Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev) branch is used to develop and test bugfixes for Marlin2ForPipetBot. After successful testing, it will be merged into Marlin2ForPipetBot.

- [6axis_PR1](https://github.com/DerAndere1/Marlin/tree/6axis_PR1) branch was merged into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/19112). This branch is now outdated. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

- [9axis_PR2](https://github.com/DerAndere1/Marlin/tree/9axis_PR2) branch was merged into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/23112).This branch is now outdated. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

- Other branches: Deprecated legacy code. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

## Hardware Abstraction Layer (HAL)

Marlin 2.0 introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility. Retaining AVR compatibility and a single code-base is important to us, because we want to make sure that features and patches get as much testing and attention as possible, and that all platforms always benefit from the latest improvements.

### Supported Platforms

  Platform|MCU|Example Boards
  --------|---|-------
  [Arduino AVR](https://www.arduino.cc/)|ATmega|RAMPS, Melzi, RAMBo
  [Teensy++ 2.0](https://www.microchip.com/en-us/product/AT90USB1286)|AT90USB1286|Printrboard
  [Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue)|SAM3X8E|RAMPS-FD, RADDS, RAMPS4DUE
  [ESP32](https://github.com/espressif/arduino-esp32)|ESP32|FYSETC E4, E4d@BOX, MRR
  [LPC1768](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1700-cortex-m3/512-kb-flash-64-kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|ARM® Cortex-M3|MKS SBASE, Re-ARM, Selena Compact
  [LPC1769](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1700-cortex-m3/512-kb-flash-64-kb-sram-ethernet-usb-lqfp100-package:LPC1769FBD100)|ARM® Cortex-M3|Smoothieboard, Azteeg X5 mini, TH3D EZBoard
  [STM32F103](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)|ARM® Cortex-M3|Malyan M200, GTM32 Pro, MKS Robin, BTT SKR Mini
  [STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html)|ARM® Cortex-M4|ARMED, Rumba32, SKR Pro, Lerdge, FYSETC S6
  [STM32F7x6](https://www.st.com/en/microcontrollers-microprocessors/stm32f7x6.html)|ARM® Cortex-M7|The Borg, RemRam V1
  [SAMD51P20A](https://www.adafruit.com/product/4064)|ARM® Cortex-M4|Adafruit Grand Central M4
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|ARM® Cortex-M4|
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|ARM® Cortex-M4|
  [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)|ARM® Cortex-M7|
  [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)|ARM® Cortex-M7|
  Linux Native|x86/ARM/etc.|Raspberry Pi

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the ([bugfix-2.1.x](https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.1.x)) branch.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin 2.0.x life-cycle.
- Follow the [Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit Feature Requests and Bug Reports to the [Issue Queue](https://github.com/MarlinFirmware/Marlin/issues/new/choose). Support resources are also listed there.
- Whenever you add new features, be sure to add tests to `buildroot/tests` and then run your tests locally, if possible.
  - It's optional: Running all the tests on Windows might take a long time, and they will run anyway on GitHub.
  - If you're running the tests on Linux (or on WSL with the code on a Linux volume) the speed is much faster.
  - You can use `make tests-all-local` or `make tests-single-local TEST_TARGET=...`.
  - If you prefer Docker you can use `make tests-all-local-docker` or `make tests-all-local-docker TEST_TARGET=...`.

## Marlin Support

The Issue Queue is reserved for Bug Reports and Feature Requests. To get help with configuration and troubleshooting, please use the following resources:

- [Marlin Documentation](https://marlinfw.org) - Official Marlin documentation
- [Marlin Discord](https://discord.gg/n5NJ59y) - Discuss issues with Marlin users and developers
- Facebook Group ["Marlin Firmware"](https://www.facebook.com/groups/1049718498464482/)
- RepRap.org [Marlin Forum](https://forums.reprap.org/list.php?415)
- Facebook Group ["Marlin Firmware for 3D Printers"](https://www.facebook.com/groups/3Dtechtalk/)
- [Marlin Configuration](https://www.youtube.com/results?search_query=marlin+configuration) on YouTube

## Contributors

Marlin is constantly improving thanks to a huge number of contributors from all over the world bringing their specialties and talents. Huge thanks are due to [all the contributors](https://github.com/MarlinFirmware/Marlin/graphs/contributors) who regularly patch up bugs, help direct traffic, and basically keep Marlin from falling apart. Marlin's continued existence would not be possible without them.


## Credits

Regular users can open and close their own issues, but only the administrators can do project-related things like add labels, merge changes, set milestones, and kick trolls. The current Marlin admin team consists of:

 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA - Project Maintainer &nbsp; [💸 Donate](https://www.thinkyhead.com/donate-to-marlin)
 - Roxanne Neufeld [[@Roxy-3D](https://github.com/Roxy-3D)] - USA
 - Keith Bennett [[@thisiskeithb](https://github.com/thisiskeithb)] - USA &nbsp; [💸 Donate](https://github.com/sponsors/thisiskeithb)
 - Peter Ellens [[@ellensp](https://github.com/ellensp)] - New Zealand
 - Victor Oliveira [[@rhapsodyv](https://github.com/rhapsodyv)] - Brazil
 - Chris Pepper [[@p3p](https://github.com/p3p)] - UK
 - Jason Smith [[@sjasonsmith](https://github.com/sjasonsmith)] - USA
 - Luu Lac [[@shitcreek](https://github.com/shitcreek)] - USA
 - Bob Kuhn [[@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)] - USA
 - Erik van der Zalm [[@ErikZalm](https://github.com/ErikZalm)] - Netherlands &nbsp; [💸 Donate](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

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