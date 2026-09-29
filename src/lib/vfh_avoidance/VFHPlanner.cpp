/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHPlanner.hpp"

#include <float.h>
#include <cmath>

#include <lib/mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

int VFHPlanner::wrapBin(int bin)
{
	// Keep circular histogram indexes in the range [0, BIN_COUNT).
	bin %= static_cast<int>(BIN_COUNT);
	return bin < 0 ? bin + BIN_COUNT : bin;
}

float VFHPlanner::angleDifference(float first, float second)
{
	return fabsf(matrix::wrap_pi(first - second));
}

int VFHPlanner::angleToBin(float angle, const Input &input) const
{
	// Convert an angle to its nearest sector across the -pi/+pi boundary.
	const float relative_angle = matrix::wrap_pi(angle - input.angle_offset);
	return wrapBin(static_cast<int>(roundf(relative_angle / input.angle_increment)));
}

VFHPlanner::Result VFHPlanner::update(const Input &input)
{
	Result result{};

	// Reject invalid geometry and require the 72 sectors to cover one full revolution.
	if (!PX4_ISFINITE(input.angle_offset) || !PX4_ISFINITE(input.angle_increment)
	    || input.angle_increment <= FLT_EPSILON || !PX4_ISFINITE(input.target_direction)
	    || !PX4_ISFINITE(input.current_direction) || _config.influence_distance <= FLT_EPSILON
	    || _config.vehicle_radius < 0.f || _config.safety_margin < 0.f
	    || _config.histogram_threshold_low < 0.f || _config.histogram_threshold_high > 1.f
	    || _config.histogram_threshold_low >= _config.histogram_threshold_high
	    || _config.max_target_direction_change <= 0.f
	    || fabsf(input.angle_increment * BIN_COUNT - 2.f * M_PI_F) > input.angle_increment * 0.5f) {
		return result;
	}

	float primary_histogram[BIN_COUNT] {};
	const float protected_radius = _config.vehicle_radius + _config.safety_margin;

	// Build the primary polar histogram. Unknown or invalid sectors are occupied
	// because an unobserved direction cannot be treated as safe free space.
	for (size_t bin = 0; bin < BIN_COUNT; ++bin) {
		if (!input.observed[bin] || !PX4_ISFINITE(input.distances[bin])
		    || input.distances[bin] <= FLT_EPSILON) {
			primary_histogram[bin] = 1.f;
			continue;
		}

		const float distance = input.distances[bin];

		if (distance >= _config.influence_distance) {
			continue;
		}

		const float density = math::constrain(1.f - distance / _config.influence_distance, 0.f, 1.f);

		// Convert the obstacle footprint into an angular interval around the ray.
		const float enlargement_angle = asinf(math::constrain(protected_radius / distance, 0.f, 1.f));
		const int enlargement_bins = static_cast<int>(ceilf(enlargement_angle / input.angle_increment));

		for (int offset = -enlargement_bins; offset <= enlargement_bins; ++offset) {
			const int affected_bin = wrapBin(static_cast<int>(bin) + offset);
			primary_histogram[affected_bin] = math::max(primary_histogram[affected_bin], density);
		}
	}

	// Values between the thresholds retain their previous state to suppress noise.
	for (size_t bin = 0; bin < BIN_COUNT; ++bin) {
		if (primary_histogram[bin] > _config.histogram_threshold_high) {
			_blocked[bin] = true;

		} else if (primary_histogram[bin] < _config.histogram_threshold_low) {
			_blocked[bin] = false;
		}
	}

	const int target_bin = angleToBin(input.target_direction, input);
	float lowest_cost = FLT_MAX;
	int selected_bin = -1;

	// Select the lowest-cost free sector while preserving mission progress and
	// avoiding abrupt changes from the current or previous direction.
	for (size_t bin = 0; bin < BIN_COUNT; ++bin) {
		if (_blocked[bin]) {
			continue;
		}

		const float candidate_direction = matrix::wrap_pi(input.angle_offset
						  + static_cast<float>(bin) * input.angle_increment);

		// Reject candidates outside the configured deviation from the target direction.
		if (angleDifference(candidate_direction, input.target_direction) > _config.max_target_direction_change) {
			continue;
		}

		result.free_sector_count++;
		float cost = _config.target_weight * angleDifference(candidate_direction, input.target_direction)
			     + _config.current_weight * angleDifference(candidate_direction, input.current_direction);

		if (_has_previous_direction) {
			cost += _config.previous_weight * angleDifference(candidate_direction, _previous_direction);
		}

		if (cost < lowest_cost) {
			lowest_cost = cost;
			selected_bin = static_cast<int>(bin);
		}
	}

	if (selected_bin < 0) {
		// The caller must stop or choose another safety action when no path exists.
		return result;
	}

	result.valid = true;
	result.avoidance_required = _blocked[target_bin];
	result.selected_direction = matrix::wrap_pi(input.angle_offset
				    + static_cast<float>(selected_bin) * input.angle_increment);
	result.cost = lowest_cost;
	_previous_direction = result.selected_direction;
	_has_previous_direction = true;
	return result;
}

void VFHPlanner::reset()
{
	// Clear hysteresis and steering history when a planning session ends.
	for (bool &blocked : _blocked) {
		blocked = false;
	}

	_has_previous_direction = false;
	_previous_direction = 0.f;
}
