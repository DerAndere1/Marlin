/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

/**
 * @file penta_axis_trt.h
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in tilting rotating table configuration.
 *
 *  A) XYZAC_TRT:
 *     This machine has a tilting table (A axis paralel to the X axis) and horizontal rotary
 *     mounted to the table (C axis).
 *  B) XYZBC_TRT:
 *     This machine has a tilting table (B axis paralel to the Y axis) and horizontal rotary
 *     mounted to the table (C axis).
 *
 * Copyright 2024 DerAndere
 *
 * Based on the file trtfuncs.c from LinuxCNC (https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/kinematics/trtfuncs.c)
 * Copyright 2016 Rudy du Preez <rudy@asmsa.co.za>
 * Author: Rudy du Preez <rudy@asmsa.co.za>
 */

#include "../core/types.h"

extern bool tool_centerpoint_control;
extern float segments_per_second;

// Center of rotation of the tilting rotating table, given as native machine coorinates when all axes are at 0.
extern float mrzp_offset_x;
extern float mrzp_offset_y;
extern float mrzp_offset_z;

// Offsets between the Centerlines of the rotational joints.
extern float rotational_offset_x; // For a machine with XYZBC axes, this is the x offset between the centerlines of the rotational joints
extern float rotational_offset_y; // For a machine with XYZAC axes, this is the y offset between the centerlines of the rotational joints
extern float rotational_offset_z; // This is the Z offset between the table surface and the joint that tilts the table

/**
 * 5 axis tilting rotary table inverse kinematics
 *
 * Calculate the joints positions for a given position, storing the result in the global delta[] array.
 * The raw position is interpreted as machine position using native_to_joint().
 */
void inverse_kinematics(const xyz_pos_t &raw);

/**
 * Calculate the joints positions for a given position.
 *
 * This is an expensive calculation.
 */
xyz_pos_t native_to_joint(const xyz_pos_t &native);
