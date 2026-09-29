/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHObstacleAdapter.hpp"

#include <float.h>
#include <climits>
#include <cmath>

#include <lib/mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

bool VFHObstacleAdapter::convert(const obstacle_distance_s &obstacle_map, uint64_t now, uint64_t timeout,
				 float frame_rotation, float target_direction, float current_direction,
				 VFHPlanner::Input &input)
{
	constexpr float expected_increment_deg = 360.f / static_cast<float>(VFHPlanner::BIN_COUNT);
	constexpr float increment_tolerance_deg = 0.01f;

	// VFH consumes the fused CP map only when its geometry and timestamp are valid.
	if (obstacle_map.frame != obstacle_distance_s::MAV_FRAME_BODY_FRD
	    || obstacle_map.timestamp == 0 || timeout == 0 || obstacle_map.timestamp > now
	    || now - obstacle_map.timestamp > timeout
	    || !PX4_ISFINITE(obstacle_map.angle_offset) || !PX4_ISFINITE(obstacle_map.increment)
	    || fabsf(obstacle_map.increment - expected_increment_deg) > increment_tolerance_deg
	    || obstacle_map.max_distance == 0 || obstacle_map.max_distance >= UINT16_MAX - 1
	    || obstacle_map.min_distance > obstacle_map.max_distance
	    || !PX4_ISFINITE(frame_rotation) || !PX4_ISFINITE(target_direction) || !PX4_ISFINITE(current_direction)) {
		return false;
	}

	// Convert the body-frame map into the fixed local-NED planning frame once.
	VFHPlanner::Input converted{};
	converted.angle_offset = matrix::wrap_pi(math::radians(obstacle_map.angle_offset) + frame_rotation);
	converted.angle_increment = math::radians(obstacle_map.increment);
	converted.target_direction = target_direction;
	converted.current_direction = current_direction;
	const uint16_t clear_distance = obstacle_map.max_distance + 1;

	for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
		const uint16_t distance = obstacle_map.distances[bin];

		// UINT16_MAX and max_distance + 1 both mean that no finite obstacle
		// distance is available for this bin. The planner treats it as occupied.
		if (distance == UINT16_MAX || distance > clear_distance) {
			converted.observed[bin] = false;
			continue;
		}

		converted.observed[bin] = true;
		converted.distances[bin] = static_cast<float>(distance) * 0.01f;
	}

	input = converted;
	return true;
}
