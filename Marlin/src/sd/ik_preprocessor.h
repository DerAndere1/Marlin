/**
 * Rep5x IK Pre-processor
 * Copyright 2025-2026 Dennis Klappe
 *
 * Pre-processes G-code file through inverse kinematics before printing,
 * eliminating real-time IK computation during print execution.
 */
#pragma once

#include "../core/types.h"

// Pre-process remaining G-code file through IK.
// Reads from the currently open SD file, writes transformed output to _iktmp.gco,
// then reopens the temp file for printing.
// Returns true on success, false on failure (original file stays open as fallback).
bool preprocess_ik_file();
