/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "VFHPlanner.hpp"

class VFHDirectionalSpeedLimiter
{
public:
	// Only the selected velocity corridor is used for directional speed control.
	static constexpr float CORRIDOR_HALF_ANGLE{0.2617993878f}; // 15 degrees

	struct Output {
		// False means the caller must not use the returned speed as a command.
		bool valid{false};
		// True means the projected corridor is already inside the hard stop distance.
		bool emergency_stop{false};
		// Closest obstacle range projected onto the selected velocity direction.
		float corridor_distance{0.f};
		// Maximum speed allowed by clearance, delay, and braking limits.
		float speed_limit{0.f};
		uint8_t considered_sector_count{0};
	};

	static Output calculate(const VFHPlanner::Input &input,
				       float selected_direction,
				       float current_speed,
				       float max_speed,
				       float brake_acceleration,
				       float delay,
				       float emergency_stop_distance,
				       float protected_radius,
				       float slowdown_distance);
};
