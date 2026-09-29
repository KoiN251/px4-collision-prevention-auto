/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "VFHPlanner.hpp"

#include <cstdint>

#include <uORB/topics/obstacle_distance.h>

class VFHObstacleAdapter
{
public:
	/**
	 * Convert a fresh 72-bin BODY_FRD map to meter/radian planner input in local NED.
	 * @param frame_rotation yaw used to rotate map angles into the planner frame
	 */
	static bool convert(const obstacle_distance_s &obstacle_map, uint64_t now, uint64_t timeout,
			    float frame_rotation, float target_direction, float current_direction,
			    VFHPlanner::Input &input);
};
