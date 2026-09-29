/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHDirectionalSpeedLimiter.hpp"

#include <float.h>
#include <cmath>

#include <lib/mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

VFHDirectionalSpeedLimiter::Output VFHDirectionalSpeedLimiter::calculate(
	const VFHPlanner::Input &input,
	const float selected_direction,
	const float current_speed,
	const float max_speed,
	const float brake_acceleration,
	const float delay,
	const float emergency_stop_distance,
	const float protected_radius,
	const float slowdown_distance)
{
	Output output{};

	// A malformed input must never produce a non-zero avoidance command.
	if (!PX4_ISFINITE(input.angle_offset) || !PX4_ISFINITE(input.angle_increment)
	    || input.angle_increment <= FLT_EPSILON || !PX4_ISFINITE(selected_direction)
	    || !PX4_ISFINITE(current_speed) || current_speed < 0.f
	    || !PX4_ISFINITE(max_speed) || max_speed <= FLT_EPSILON
	    || !PX4_ISFINITE(brake_acceleration) || brake_acceleration <= FLT_EPSILON
	    || !PX4_ISFINITE(delay) || delay < 0.f
	    || !PX4_ISFINITE(emergency_stop_distance) || emergency_stop_distance <= FLT_EPSILON
	    || !PX4_ISFINITE(protected_radius) || protected_radius < 0.f
	    || !PX4_ISFINITE(slowdown_distance) || slowdown_distance < 0.f) {
		return output;
	}

	float nearest_projected_distance = FLT_MAX;

	// Check only the angular corridor that the selected velocity will enter.
	for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
		const float sector_angle = input.angle_offset + static_cast<float>(bin) * input.angle_increment;
		const float angle_difference = fabsf(matrix::wrap_pi(sector_angle - selected_direction));

		if (angle_difference > CORRIDOR_HALF_ANGLE) {
			continue;
		}

		++output.considered_sector_count;

		// Unknown data inside the selected corridor is not safe to use for speed control.
		if (!input.observed[bin] || !PX4_ISFINITE(input.distances[bin])
		    || input.distances[bin] <= FLT_EPSILON) {
			return output;
		}

		// Use the obstacle range projected onto the actual VFH velocity direction.
		const float projected_distance = input.distances[bin] * cosf(angle_difference);
		nearest_projected_distance = math::min(nearest_projected_distance, projected_distance);
	}

	if (output.considered_sector_count == 0 || !PX4_ISFINITE(nearest_projected_distance)) {
		return output;
	}

	output.valid = true;
	output.corridor_distance = nearest_projected_distance;
	const float required_stop_distance = math::max(emergency_stop_distance, protected_radius);
	const float effective_slowdown_distance = math::max(slowdown_distance, required_stop_distance);

	if (nearest_projected_distance <= required_stop_distance) {
		output.emergency_stop = true;
		return output;
	}

	// Reserve distance for sensor/controller delay before the braking calculation.
	const float distance_after_delay = nearest_projected_distance - required_stop_distance
					   - current_speed * delay;

	if (distance_after_delay <= 0.f) {
		// There is no braking distance left after the sensor/controller delay.
		// Do not replace this safe zero with a minimum forward command.
		output.speed_limit = 0.f;
		return output;
	}

	const float physics_speed_limit = sqrtf(math::max(0.f,
										 2.f * brake_acceleration * distance_after_delay));

	// Begin a smooth speed reduction at CP_AUTO_DIST and reach the hard-stop
	// threshold at CP_VFH_STOP_DIST. The physics limit remains the lower bound.
	const float speed_range = math::max(effective_slowdown_distance - required_stop_distance, FLT_EPSILON);
	const float distance_scale = math::constrain(
		(nearest_projected_distance - required_stop_distance) / speed_range, 0.f, 1.f);
	const float distance_speed_limit = max_speed * distance_scale;

	output.speed_limit = math::min(max_speed, math::min(physics_speed_limit, distance_speed_limit));
	return output;
}
