/**
 * Marlin2ForPipetBot [https://github.com/DerAndere1/Marlin]
 * Copyright 2025 DerAndere and other Marlin2ForPipetBot authors [https://github.com/DerAndere1/Marlin]
 *
 * Based on:
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
 * @file penta_axis_head_head.h
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in head-headconfiguration.
 *
 * This machine has a tilting head and a horizontal rotary table.
 *
 * Copyright 2025 DerAndere
 *
 * Based on LinuxCNC file 5axiskins.c and Rep5x file inverse-kinematics.js:
 * 
 * Based on LinuxCNC file 5axiskins.c
 * Author: Chris Radek <chris@timeguy.com>
 *
 * Copyright (c) 2007 Chris Radek
 * 
 * Based on Rep5x file inverse-kinematics.js
 * Author: Dennis Klappe
 * 
 * Copyright 2025 Dennis Klappe
 *
 */

#include "../core/types.h"

extern bool tool_centerpoint_control;
extern float segments_per_second;

// Center of rotation of the tilting rotating table, given as native machine coorinates when all axes are at 0.
extern float mrzp_offset_z;

// Offsets between the Centerlines of the rotational joints.
extern float rotational_offset_y; // For a machine with XYZBC axes, this is the y offset between the centerlines of the rotational joints

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

void forward_kinematics(const xyz_pos_t &joint_pos);

/**
 * Calculate the positions for a given joint position.
 *
 * This is an expensive calculation.
 */
xyz_pos_t joint_to_native(const xyz_pos_t &joint_pos);
