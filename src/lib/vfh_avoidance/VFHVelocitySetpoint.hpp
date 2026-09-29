/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "VFHPlanner.hpp"

#include <matrix/matrix/math.hpp>

class VFHVelocitySetpoint
{
public:
	/**
	 * Convert a selected heading and speed into a horizontal velocity setpoint.
	 * Returns false when the planner result or requested speed is invalid.
	 */
	static bool generate(const VFHPlanner::Result &result, float frame_rotation, float speed,
			     matrix::Vector2f &velocity_setpoint);
};
