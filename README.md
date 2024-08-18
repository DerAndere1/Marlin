# Marlin2ForPipetBot 3D Printer and Lab Robot CNC Firmware
 
Additional documentation can be found in the repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot), the [Wiki](https://github.com/DerAndere1/Marlin/wiki) or on the [PipetBot-A8 project homepage](https://derandere.gitlab.io/pipetbot-a8) that is part of the [authors homepage](https://derandere.gitlab.io). 
For multi-axis CNC machines and lab robots (liquid handling robots, "pipetting robots"). Please test this firmware and let us know if it misbehaves in any way. Volunteers are standing by!

Marlin2ForPipetBot supports up to nine non-extruder axes plus extruders (e.g. XYZABCUVW+E or XYZABCW+E or XYZCUVW+E or XYZABC+E or XYZUVW+E). In addition, support for a laser and a spindle is provided.

## G-code
The G-code syntax of Marlin2ForPipetBot closely resembles that of LinuxCNC (the successor of NIST RS274NGC interpreter - version 3). Here is a list of G-codes that deviated in official MarlinFirmware/Marlin and that are brought more in line with LinuxCNC syntax:
- F (feedrate for G0, G1, G2, G3, G4, G5, G81, G82, G83)
- G10 (set offsets)

### G1 (Linear Move)

#### Usage

Example syntax for movement (G-code G1) with 9 axes plus extruder (default axis names: XYZABCUVW+E): 
```
G1 [Xx.xxxx] [Yy.yyyy] [Zz.zzzz] [Aa.aaaa] [Bb.bbbb] [Cc.cccc] [Uu.uuuu] [Vv.vvvv] [Ww.wwww] [Ee.eeee] [Ff.ffff]
```

#### Parameters:

##### `X`, `Y`, `Z`

Position in the cartesian coordinate system consisting of primary linear axes X, Y and Z. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

##### `A`, `B`, `C`

Angular position in the pseudo-cartesian coordinate system consisting of rotational axes A, B, and C that are parallel to axes X, Y and Z, respectively. Unit: degrees

##### `U`, `V`, `W`

Position in the cartesian coordinate system consisting of secondary linear axes U, V and W that are parallel to axes X, Y and Z. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

##### `E`

Distance the E stepper should move. Unit: mm (after G-code G21) or imperial inch (after G-code G20)

##### `F`

Feedrate as defined by LinuxCNC:

- For motion involving one or more of the X, Y, and Z axes (with or without motion of other axes), the feed rate means length units per minute along the programmed XYZ path, as if the other axes were not moving.
- For motion involving one or more of the secondary linear axes (axis names 'U', 'V', or 'W') with the X, Y , and Z axes not moving (with or without motion of rotational axes), the feed rate means length units per minute along the programmed UVW path (using the usual Euclidean metric in the UVW coordinate system), as if the rotational axes were not moving.
- For motion involving one or more of the rotational axes (axis names 'A', 'B' or 'C') with linear axes not moving, the rate is applied as follows. Let `dA`, `dB`, and `dC` be the angles of rotation around axes A, B, and C axes, respectively, in units of degrees. Let `D = sqrt((dA)^2 + (dB)^2 + (dC)^2)`. Conceptually, `D` is a measure of total angular motion, using the usual Euclidean metric. The motion involving rotational axes should be a coordinated linear motion so that the elapsed total time (in minutes) from the start to the end of the motion is `T = D / F` plus any time required for acceleration or deceleration.

To change the feed rate interpretation, the option `ARTICULATED_ROBOT_ARM` can be defined. With that option enabled, feed reference are all axes. This means that in all cases all axes are moved in coordinated linear motion so that the time (in minutes) required for the move is `T = sqrt((dA)^2 + (dB)^2 + (dC)^2 + (dU)^2 + (dV)^2 + (dW)^2) / F` plus any time for acceleration or deceleration.

### G10 (Set offsets)

Set offsets. See the following references:
- https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G10-L1_
- https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G10-L2_
- https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G10-L11
- https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G10-L20


### M665 (PENTA_AXIS configuration)

Configure `PENTA_AXIS_TRT` and `PENTA_AXIS_HT` geometry values.

#### Usage

`M665 [X<MRZP-offset>] [Y<MRZP-offset>] [Z<MRZP-offset>] [S<segments-per-second>] [I<rotational-joint-offset>] [J<rotational-joint-offset>] [K<rotational-joint-offset>]`

#### Parameters

##### `Z<MRZP-offset>`

Set the machine rotary zero point (MRZP) Z offset. 
- For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Z axis from machine zero point to the center of rotation. The center of rotation is usually the center of the top surface of the table.
- For 5 axis CNC machines in head-table configuration (PENTA_AXIS_HT) this is the distance along the Z axis from the machine zero point to the horizontal centerline of the joint that tilts the tool head when all axes are in zero position.

See `DEFAULT_MRZP_OFFSET_Z` and see the definition of the pivot point (`Pz`) in reference https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

##### `X<MRZP-offset>`

Set the machine rotary zero point (MRZP) X offset.
For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the X axis from machine zero point to the center of rotation when all axes are in neutral (zero) position. The center of rotation is usually the center of the top surface of the table.

See `DEFAULT_MRZP_OFFSET_X` and see the definition of the pivot point (`Px`) in reference https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html


##### `Y<MRZP-offset>`

Set the machine rotary zero point (MRZP) Y offset. 
For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Y axis from machine zero point to the center of rotation when all axes are in neutral (zero) position. The center of rotation is usually the center of the top surface of the table.

See the definition of the pivot point (`Py`) in reference https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

##### `I<rotational-joint-offset>`

Set the rotational joint X offset. For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) with XYZBC axes this is the distance along the X axis from the vertical centerline of the joint of the rotary table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See `DEFAULT_ROTATIONAL_JOINT_OFFSET_X`. Also, see definition of `Dx` in sections "5.3. Transformations for a xyzbc-trt machine with rotary axis offsets" and "7. Custom Kinematics Components" in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

##### `J<rotational-joint-offset>`

Set the rotational joint Y offset. For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) with XYZAC axes this is the distance along the Y axis from the vertical centerline of the joint of the rotary table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See `DEFAULT_ROTATIONAL_JOINT_OFFSET_Y`. Also, see definition of `Dy` in sections "5.3. Transformations for a xyzbc-trt machine with rotary axis offsets" and "7. Custom Kinematics Components" in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

##### `K<rotational-joint-offset>`

Set the rotational joint Z offset. For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Z axis from the surface at the top of the table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See `DEFAULT_ROTATIONAL_JOINT_OFFSET_Z`. Also, see definition of `Dz` in sections "5.3. Transformations for a xyzbc-trt machine with rotary axis offsets" and "7. Custom Kinematics Components" in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html


### M211 (Software Endstops)

Set whether printing should abort or moves should be clamped in the event of any software endstop being triggered. This provides a fast way to abort a print in the event of mechanical failure such as loose couplings, lost steps, diverted axes, binding, etc., which lead to axes being very far out of position.

#### Notes

- Requires `SOFTWARE_ENDSTOPS_*` for at least one axis.

- Use `ENDSTOPS_ALWAYS_ON_DEFAULT` or `M120` to ensure that monitoring of limit switches is enabled.

#### Usage

`M211 S<flag> H<flag>`

#### Parameters

##### `S<flag>`

Whether (1) or not (0) to enable software endstops.

##### `H<flag>`

Whether to abort machining on software endstops hit (1) or whether to clamp moves to the software endstops (0). Requires `ABORT_ON_SOFTWARE_ENDSTOP`

## Configuration

Configuration is done by editing the file Marlin/Configuration.h. E.g. change

`//#define I_DRIVER_TYPE A4988`

to: 

`#define I_DRIVER_TYPE A4988`

The following is a list of options in Marlin2ForPipetBot that are relevant for machines with more than three axes or that deviate from those of MarlinFirmware/Marlin:

### `X_DRIVER_TYPE`

`X_DRIVER_TYPE`, `Y_DRIVER_TYPE`, `Z_DRIVER_TYPE`, `I_DRIVER_TYPE`, `J_DRIVER_TYPE`, `K_DRIVER_TYPE`, `U_DRIVER_TYPE`, `V_DRIVER_TYPE`, `W_DRIVER_TYPE`: These settings allow Marlin to tune stepper driver timing and enable advanced options for stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 
Use TMC2208/TMC2208_STANDALONE for TMC2225 drivers and TMC2209/TMC2209_STANDALONE for TMC2226 drivers.

Each driver is associated with an axis (internal axis identifiers: 
X, Y, Z, I, J, K, U, V, W) or an extruder (E0 to E7). 
Each axis gets its own stepper control and endstops depending on the following settings:
`[[I, [J, [K...]]]_STEP_PIN`, `[I, [J, [K...]]]_ENABLE_PIN`, `[I, [J, [K...]]]_DIR_PIN`,
`[I, [J, [K...]]]_STOP_PIN`, `USE_[I, [J, [K...]]][MIN || MAX]_PLUG`, 
`[I, [J, [K...]]]_ENABLE_ON`, `DISABLE_[I, [J, [K...]]]`, `[I, [J, [K...]]]_MIN_POS`, 
`[I, [J, [K...]]]_MAX_POS`, `[I, [J, [K...]]]_HOME_DIR`, possibly `DEFAULT_[I, [J, [K...]]]JERK`, 
and definition of the respective values of `DEFAULT_AXIS_STEPS_PER_UNIT`, `DEFAULT_MAX_FEEDRATE`,
`DEFAULT_MAX_ACCELERATION`, `HOMING_FEEDRATE_MM_M`, `AXIS_RELATIVE_MODES`, `MICROSTEP_MODES`,
`HOMING_BUMP_MM`, `HOMING_BUMP_DIVISOR`, and possibly also values of
`MAX_FEEDRATE_EDIT_VALUES`, `MAX_ACCEL_EDIT_VALUES`, `MAX_JERK_EDIT_VALUES`, `MANUAL_FEEDRATE`, 
`SENSORLESS_BACKOFF_MM`, `HOMING_BACKOFF_POST_MM`, `BACKLASH_DISTANCE_MM`.
`NOZZLE_TO_PROBE_OFFSETS` has to be extended with elemets of value 0
until the number of elements is equal to the value of `NUM_AXES`.

Allowed values: [A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01, TB6560, TB6600, TMC2100, TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE, TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE, TMC26X, TMC26X_STANDALONE, TMC2660, TMC2660_STANDALONE, TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE]

### `AXIS4_ROTATES`

`AXIS4_ROTATES`, `AXIS5_ROTATES`, `AXIS6_ROTATES`, `AXIS7_ROTATES`, `AXIS8_ROTATES`, `AXIS9_ROTATES`:
If enabled, the corresponding axis is a rotational axis for which positions are specified in angular degrees.
For moves involving only rotational axes, feedrate is interpreted in angular degrees.

### `AXIS4_NAME`

`AXIS4_NAME`, `AXIS5_NAME`, `AXIS6_NAME`, `AXIS7_NAME`, `AXIS8_NAME`, `AXIS9_NAME`:
Axis codes for additional axes:
This defines the axis code that is used in G-code commands to reference a specific axis. Conventional axis names are as follows:
   * 'A' for rotational axis parallel to X
   * 'B' for rotational axis parallel to Y
   * 'C' for rotational axis parallel to Z
   * 'U' for secondary linear axis parallel to X
   * 'V' for secondary linear axis parallel to Y
   * 'W' for secondary linear axis parallel to Z

Regardless of the settings, firmware-internal axis names are
I (AXIS4), J (AXIS5), K (AXIS6), U (AXIS7), V (AXIS8), W (AXIS9).

Allowed values: ['A', 'B', 'C', 'U', 'V', 'W'] 

### `ARTICULATED_ROBOT_ARM`

When enabled, feed rate (the F-word in G1 G-code commands) is interpreted with all axes as the feed reference. For compatibility with Marlin <= 2.0.9.3, grblHAL/grblHAL-core, Duet3D/RepRap-Firmware, synthetos/g2core and for compatibility with articulated robots (robot arms) for which inverse kinematics are not yet implemented in Marlin. For a detailed description of feedrate, see first section.

### `FOAMCUTTER_XYUV`

Define `FOAMCUTTER_XYUV` kinematics for a hot wire cutter with parallel horizontal axes X, U where the hights of the two wire ends are controlled by parallel axes Y, V. Currently only works with `*_DRIVER_TYPE` defined for 5 axes (X, Y, Z, I and J). A dummy pin number can be assigned to pins for the unused Z axis. Leave `FOAMCUTTER_XYUV` disabled for default behaviour (stepper velocities are calculated using multidimensional linear interpolation over all axes). Host software and CAM software for 4 axis foam cutters can be found at https://rckeith.co.uk/file-downloads/ and https://www.jedicut.com/en/download/ .

### `PENTA_AXIS_TRT`

Define `PENTA_AXIS_TRT` kinematics for a 5 axis CNC machine in tilting-rotating-table configuration to enable tool center point control. These machines have 3 mutually orthogonal prismatic ("linear") joints aligned with axes XYZ plus a rotary table (C axis) mounted on a tilting table (A or B axis). Two different machine geometries of this type are supported:
- XYZAC TRT machine: The rotational joint of the tilting table is parallel to the X axis and the joint that rotates the table is parallel to the Z axis when all axes are at zero position. This requires `AXIS4_NAME 'A'` and `AXIS5_NAME 'C'`.
- XYZBC TRT machine: The rotational joint of the tilting table is parallel to the Y axis and the joint that rotates the table is parallel to the Z axis when all axes are at zero position. This requires `AXIS4_NAME 'B'`and `AXIS5_NAME 'C'` (see section `AXIS4_NAME`).

### `PENTA_AXIS_HT`

Define `PENTA_AXIS_HT` kinematics for a 5 axis CNC machine in head-table configuration to enable tool center point control. These machines have 3 mutually orthogonal prismatic ("linear") joints aligned with axes XYZ plus a swivel head (A or B axis) and a horizontal rotary table (C axis). There are two possible machine geometries:
- XYZAC head-table machine: The rotational joint of the swivel head is parallel to the X axis when all axes are at zero position. This requires `AXIS4_NAME 'A'` and `AXIS5_NAME 'C'`.
- XYZBC head-table machine: The rotational joint of the swivel head is parallel to the Y axis when all axes are at zero position. This requires `AXIS4_NAME 'B'`and `AXIS5_NAME 'C'` (see section `AXIS4_NAME`).

### `DEFAULT_MRZP_OFFSET_Z`

Machine rotary zero point (MRZP) Z offset. 
- For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Z axis from machine zero point to the center of rotation. The center of rotation is usually the center of the top surface of the table when all axes are in zero position.
- For 5 axis CNC machines in head-table configuration (PENTA_AXIS_HT) this is the distance along the Z axis from the machine zero point to the horizontal centerline of the joint that tilts the tool head when all axes are in zero position.

See the definition of the pivot point (`Pz`) in this reference: 
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `DEFAULT_MRZP_OFFSET_X`

Machine rotary zero point (MRZP) X offset.
For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the X axis from machine zero point to the center of rotation when all axes are in neutral (zero) position. The center of rotation is usually the center of the top surface of the table.

See the definition of the pivot point (`Px`) in this reference: 
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `DEFAULT_MRZP_OFFSET_Y`

Machine rotary zero point (MRZP) Y offset. 
For 5 axis CNC machines with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Y axis from machine zero point to the center of rotation when all axes are in neutral (zero) position. The center of rotation is usually the center of the top surface of the table.

See the definition of the pivot point (`Pz`) in this reference: 
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `DEFAULT_ROTATIONAL_JOINT_OFFSET_X`

For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) with XYZBC axes this is the distance along the X axis from the vertical centerline of the joint of the rotary table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See definition of `Dx` in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `DEFAULT_ROTATIONAL_JOINT_OFFSET_Y`

For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) with XYZAC axes this is the distance along the Y axis from the vertical centerline of the joint of the rotary table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See definition of `Dy` in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `DEFAULT_ROTATIONAL_JOINT_OFFSET_Z`

For a 5 axis CNC machine with a tilting rotary table (PENTA_AXIS_TRT) this is the distance along the Z axis from the surface at the top of the table to the horizontal centerline of the joint that tilts the table. Measured when all rotational axes are in neutral (zero) position so that the table is oriented horizontally.

See definition of `Dz` in this reference:
- https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html

### `TOOLS`

Number of tools, including extruders. Tool indices, starting with 0, must be assigned in the following order: extruders (requires EXTRUDERS > 0), laser (requires `LASER_FEATURE`), anf finally tools for a spindle (requires `SPINDLE_FEATURE`). Offsets of each tool from tool 0 must be defined with `HOTEND_OFFSET_X`, `HOTEND_OFFSET_Y` and `HOTEND_OFFSET_Z`.

### `ABORT_ON_SOFTWARE_ENDSTOPS`

Abort printing when any software endstop is triggered. This feature is enabled with 'M211 H1' or from the LCD menu.
Software endstops must be activated for this option to work.

### `SAFE_BED_LEVELING_START_X`

`SAFE_BED_LEVELING_START_X`, `SAFE_BED_LEVELING_START_Y`, `SAFE_BED_LEVELING_START_Z`, `SAFE_BED_LEVELING_START_I`, `SAFE_BED_LEVELING_START_J`, `SAFE_BED_LEVELING_START_K`, `SAFE_BED_LEVELING_START_U`, `SAFE_BED_LEVELING_START_V`, `SAFE_BED_LEVELING_START_W`: 
Safe bed leveling start coordinates. If enabled, the respective axis is moved to the specified position at the beginning of the bed leveling procedure.
Required for multi-axis machines (`I_DRIVER_TYPE` ... defined).
Values must be chosen so that the bed is oriented horizontally and so that the Z-probe is oriented vertically.
Note: If inverse kinematics for your machine are not implemented, bed leveling produces wrong results for all moves where the bed is not oriented horizontally or where the tool head is not oriented vertically. In these cases, bed leveling must be disabled.

### `LCD_SHOW_SECONDARY_AXES`

Show the position of secondary axes I[J[K]] instead of icons on an DOGM LCD (e.g. REPRAP_FULL_GRAPHICS_DISPLAY).

### `QUICK_HOME_SECONDARY_AXES`

If all axes are homed, first raise Z, then move all axes except Z simultaneously to their home position. Once the first axis reaches its home position, the axes will be homed individually in sequence XYZIJKUVW. Requires `QUICK_HOME`.

### `CLASSIC_JERK`

For multi-axis machines it is highly recommended to enable `CLASSIC_JERK`.

## Marlin2ForPipetBot Branch

__Not for production use. Use with caution!__

Marlin2forPipetBot is a branch of the Marlin fork by DerAndere (based on 
https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789). 

This branch is for patches to the latest Marlin2ForPipetBot release version.

## Example Configurations

Before you can build Marlin for your machine you'll need a configuration for your specific hardware. Upon request, your vendor will be happy to provide you with the complete source code and configurations for your machine, but you'll need to get updated configuration files if you want to install a newer version of Marlin. Fortunately, Marlin users have contributed dozens of tested configurations to get you started. Visit the [MarlinFirmware/Configurations](https://github.com/MarlinFirmware/Configurations) repository to find the right configuration for your hardware.

## Building Marlin2ForPipetBot

To build Marlin2ForPipetBot you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The Marlin team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html). [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) is preconfigured for the Anet-V1.0 board of the PipetBot-A8. When using the default build environment (`default_env = melzi_optiboot`), upload of the compiled Marlin2ForPipetBot firmware to the board via USB using the optiboot bootloader requires burning of the [optiboot bootloader](https://github.com/Optiboot/optiboot) onto the board as described in the [SkyNet3D/anet-board documentation](https://github.com/SkyNet3D/anet-board).

The different branches in the git repository https://github.com/DerAndere1/Marlin reflect different stages of development: 
 
- [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) branch is the stable release branch for [tagged releases of Marlin2ForPipetBot firmware](https://github.com/DerAndere1/Marlin/tags). It is optimized and preconfigured for the [PipetBot-A8](https://derandere.gitlab.io/pipetbot-a8) by default. Currently it is based on MarlinFirmware/Marlin bugfix-2.1.x from 2022-06-07, [https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789](https://github.com/MarlinFirmware/Marlin/tree/3e9fb34892e85bc4069acf5baddbf12d6cd47789) or later. In addition to MarlinFirmware/Marlin's support for 9 axes, including rotational axes (`AXISn_ROTATES`), it adds simultaneous homing of all axes except Z (`QUICK_HOME_ALL_NON_Z_AXES`), a second controller fan pin (`CONTROLLER_FAN2_PIN`), and it can be configured to show positions of secondary axes on an LCD (`LCD_SHOW_SECONDARY_AXES`).

- [Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev) branch is used to develop and test bugfixes for Marlin2ForPipetBot. After successful testing, it will be merged into Marlin2ForPipetBot.

- [6axis_PR1](https://github.com/DerAndere1/Marlin/tree/6axis_PR1) branch was merged into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/19112). This branch is now outdated. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

- [9axis_PR2](https://github.com/DerAndere1/Marlin/tree/9axis_PR2) branch was merged into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/23112).This branch is now outdated. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

- Other branches: Deprecated legacy code. Use current [MarlinFirmware/Marlin](https://github.com/MarlinFirmware/Marlin) instead.

## Hardware Abstraction Layer (HAL)

Marlin includes an abstraction layer to provide a common API for all the platforms it targets. This allows Marlin code to address the details of motion and user interface tasks at the lowest and highest levels with no system overhead, tying all events directly to the hardware clock.

Every new HAL opens up a world of hardware. At this time we need HALs for RP2040 and the Duet3D family of boards. A HAL that wraps an RTOS is an interesting concept that could be explored. Did you know that Marlin includes a Simulator that can run on Windows, macOS, and Linux? Join the Discord to help move these sub-projects forward!

## 8-Bit AVR Boards

A core tenet of this project is to keep supporting 8-bit AVR boards while also maintaining a single codebase that applies equally to all machines. We want casual hobbyists to benefit from the community's innovations as much as possible just as much as those with fancier machines. Plus, those old AVR-based machines are often the best for your testing and feedback!

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
  [STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html)|ARM® Cortex-M4|ARMED, Rumba32, SKR Pro, Lerdge, FYSETC S6, Artillery Ruby
  [STM32F7x6](https://www.st.com/en/microcontrollers-microprocessors/stm32f7x6.html)|ARM® Cortex-M7|The Borg, RemRam V1
  [STM32G0B1RET6](https://www.st.com/en/microcontrollers-microprocessors/stm32g0x1.html)|ARM® Cortex-M0+|BigTreeTech SKR mini E3 V3.0
  [STM32H743xIT6](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html)|ARM® Cortex-M7|BigTreeTech SKR V3.0, SKR EZ V3.0, SKR SE BX V2.0/V3.0
  [SAMD51P20A](https://www.adafruit.com/product/4064)|ARM® Cortex-M4|Adafruit Grand Central M4
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|ARM® Cortex-M4|
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|ARM® Cortex-M4|
  [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)|ARM® Cortex-M7|
  [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)|ARM® Cortex-M7|
  Linux Native|x86/ARM/etc.|Raspberry Pi

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the ([Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev)) branch. I sometimes rebase that branch wich results in merge conflicts when you submit a Pull Request. Please ignore merge conflics and just submit the Pull Request with unresolved conflicts. I will take care of it.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin2ForPipetBot life-cycle.
- Follow the [Coding Standards](https://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit Feature Requests and Bug Reports to the [Issue Queue](https://github.com/DerAndere1/Marlin/issues). Support resources are also listed there.
- Whenever you add new features, be sure to add tests to `buildroot/tests` and then run your tests locally, if possible.
  - It's optional: Running all the tests on Windows might take a long time, and they will run anyway on GitHub.
  - If you're running the tests on Linux (or on WSL with the code on a Linux volume) the speed is much faster.
  - You can use `make tests-all-local` or `make tests-single-local TEST_TARGET=...`.
  - If you prefer Docker you can use `make tests-all-local-docker` or `make tests-all-local-docker TEST_TARGET=...`.

## Marlin Support

The Issue Queue is reserved for Bug Reports and Feature Requests. To get help with configuration and troubleshooting, please use the following resources:

- [Marlin Documentation](https://marlinfw.org) - Official Marlin documentation
- [Multi-Axis-Marlin Wiki](https://github.com/DerAndere1/Marlin/wiki) - Information related to machines with more than 3 axes
- [Marlin Configuration](https://www.youtube.com/results?search_query=marlin+configuration) on YouTube
- [Marlin2ForPipetBot issue queue](https://github.com/DerAndere1/Marlin/issues)

## Contributors

Marlin is constantly improving thanks to a huge number of contributors from all over the world bringing their specialties and talents. Huge thanks are due to [all the contributors](https://github.com/DerAndere1/Marlin/graphs/contributors) who regularly patch up bugs, help direct traffic, and basically keep Marlin from falling apart. Marlin's continued existence would not be possible without them.


## Credits

Marlin2ForPipetBot (https://github.com/DerAndere1/Marlin)

Copyright 2024 DerAndere

Marlin2ForPipetBot is modified by:

 - DerAndere [[@DerAndere1](https://github.com/DerAndere1)] - Germany - Marlin2ForPipetBot Project Maintainer &nbsp; [💸 Donate](https://www.paypal.com/donate/?hosted_button_id=TNGG65GVA9UHE)
 - Garbriel Beraldo [@GabrielBeraldo](https://github.com/GabrielBeraldo) - Brasil
 - Olivier Briand [@hobiseven](https://github.com/hobiseven) - France
 - Wolverine [@MohammadSDGHN](https://github.com/MohammadSDGHN) - Undisclosed
 - bilsef [@bilsef](https://github.com/bilsef) - Undisclosed
 - FNeo31 [@FNeo31](https://github.com/FNeo31) - Undisclosed
 - HendrikJan-5D [@HendrikJan-5D](https://github.com/HendrikJan-5D) - Undisclosed
 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA - MarlinFirmware Maintainer &nbsp; [💸 Donate](https://www.thinkyhead.com/donate-to-marlin)


Marlin2ForPipetBot is based on: 

MarlinFirmware/Marlin Marlin 3D Printer Firmware (https://github.com/MarlinFirmware/Marlin)

Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]

### Contributions

|Author | Contact | Contribution |
|-------|---------|--------------|
| DerAndere | [@DerAndere1](https://github.com/DerAndere1) | main developer of multi-axis support, idea, initial implementation, added features `AXIS4_NAME`, `I_DRIVER_TYPE` ... `W_DRIVER_TYPE`, `PENTA_AXIS_HT`, `PENTA_AXIS_TRT`, `TOOLS`, `EVENT_GCODE_TOOLCHANGE_T0`, `EVENT_GCODE_TOOLCHANGE_T1`, `SAFE_BED_LEVELING_POSITION_X`, `AXIS4_ROTATES`, `FOAMCUTTER_XYUV`, `ARTICULATED_ROBOT_ARM`, `QUICK_HOME_ALL_NON_Z_AXES`, `LCD_SHOW_SECONDARY_AXES`, added commands `G10`, `G43`, `G43.4`, `G49`, bugfixes |
| Gabriel Beraldo | [@GabrielBeraldo](https://github.com/GabrielBeraldo) | hardware debugging, bugfixes yielding first working prototype (make additional axes move, fix EEPROM) |
| Olivier Briand | [@hobiseven](https://github.com/hobiseven) | testing, added experimental compatibility with different configurations, code review, `FOAMCUTTER_XYUV` feed rate interpretation mode |
| Wolverine | [@MohammadSDGHN](https://github.com/MohammadSDGHN) | added experimental compatibility with different configurations (BigTreeTech SKR Pro 1.1, Trinamic TMC drivers, StealthChop, sensorless homing) |
| bilsef | [@bilsef](https://github.com/bilsef) | testing, code review, bugfixes (fix movement of additional axes, fix types.h, fix EEPROM)|
| FNeo31 | [@FNeo31](https://github.com/FNeo31) | added experimental drilling cyles (G81, G82, G83) |
| HendrikJan-5D | [@HendrikJan-5D](https://github.com/HendrikJan-5D) | testing bed leveling and Trinamic TMC, bugfixes yielding first working 9 axis printer prototype |
| Paloky | [@paloky](https://github.com/paloky) | Initial extension of multi-axis support from 6 to 8 axes |
| Keith | [@rcKeith](https://github.com/rcKeith) | Testing of `FOAMCUTTER_XYUV` and `LCD_SHOW_SECONDARY_AXES` |
| Phillipp Webb | [@Domush](https://github.com/Domush) | Added commands G10 L2, G10 L20 |
| Scott Lahteine | [@thinkyhead](https://github.com/thinkyhead) | code review, refactoring | 
| MarlinFirmware | [@MarlinFirmware](https://github.com/MarlinFirmware/Marlin) | Marlin 3D Printer firmware |


## License

Marlin2ForPipetBot is published under the [GPL license](https://github.com/DerAndere1/Marlin/blob/Marlin2ForPipetBot/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
