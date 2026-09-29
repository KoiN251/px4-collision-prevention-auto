/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHVelocitySetpoint.hpp"

	#include <float.h>
#include <cmath>

#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

bool VFHVelocitySetpoint::generate(const VFHPlanner::Result &result, float frame_rotation, float speed,
				   matrix::Vector2f &velocity_setpoint)
{
	if (!result.valid || !PX4_ISFINITE(result.selected_direction)
	    || !PX4_ISFINITE(frame_rotation) || !PX4_ISFINITE(speed) || speed <= FLT_EPSILON) {
		return false;
	}

	// The planner direction is already in local NED. Keep frame_rotation in the
	// API for callers that need to express the result in another frame.
	const float selected_direction_local = matrix::wrap_pi(result.selected_direction + frame_rotation);
	velocity_setpoint = matrix::Vector2f(cosf(selected_direction_local), sinf(selected_direction_local)) * speed;
	return velocity_setpoint.isAllFinite();
}
