/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <climits>
#include <cmath>

#include <lib/mathlib/math/Functions.hpp>

#include "VFHObstacleAdapter.hpp"

class VFHObstacleAdapterTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_obstacle_map.timestamp = 900'000;
		_obstacle_map.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
		_obstacle_map.min_distance = 20;
		_obstacle_map.max_distance = 1'000;
		_obstacle_map.angle_offset = -180.f;
		_obstacle_map.increment = 5.f;

		for (uint16_t &distance : _obstacle_map.distances) {
			distance = _obstacle_map.max_distance + 1;
		}
	}

	bool convert()
	{
		return VFHObstacleAdapter::convert(_obstacle_map, 1'000'000, 200'000, 0.f, 0.f, 0.f, _input);
	}

	obstacle_distance_s _obstacle_map{};
	VFHPlanner::Input _input{};
};

TEST_F(VFHObstacleAdapterTest, ConvertsGeometryAndCentimetersToMeters)
{
	_obstacle_map.distances[36] = 250;
	ASSERT_TRUE(convert());
	EXPECT_NEAR(_input.angle_offset, -M_PI_F, 1e-5f);
	EXPECT_NEAR(_input.angle_increment, math::radians(5.f), 1e-5f);
	EXPECT_TRUE(_input.observed[36]);
	EXPECT_NEAR(_input.distances[36], 2.5f, 1e-5f);
}

TEST_F(VFHObstacleAdapterTest, TreatsClearReadingAsObserved)
{
	ASSERT_TRUE(convert());
	EXPECT_TRUE(_input.observed[0]);
	EXPECT_NEAR(_input.distances[0], 10.01f, 1e-5f);
}

TEST_F(VFHObstacleAdapterTest, RotatesBodyGeometryIntoRequestedFrame)
{
	ASSERT_TRUE(VFHObstacleAdapter::convert(_obstacle_map, 1'000'000, 200'000,
			math::radians(60.f), math::radians(20.f), math::radians(10.f), _input));
	EXPECT_NEAR(_input.angle_offset, math::radians(-120.f), 1e-5f);
	EXPECT_NEAR(_input.target_direction, math::radians(20.f), 1e-5f);
	EXPECT_NEAR(_input.current_direction, math::radians(10.f), 1e-5f);
}

TEST_F(VFHObstacleAdapterTest, TreatsUnknownAndOutOfRangeReadingsAsUnobserved)
{
	_obstacle_map.distances[10] = UINT16_MAX;
	_obstacle_map.distances[11] = _obstacle_map.max_distance + 2;
	ASSERT_TRUE(convert());
	EXPECT_FALSE(_input.observed[10]);
	EXPECT_FALSE(_input.observed[11]);
}

TEST_F(VFHObstacleAdapterTest, KeepsZeroDistanceAsObservedDanger)
{
	_obstacle_map.distances[36] = 0;
	ASSERT_TRUE(convert());
	EXPECT_TRUE(_input.observed[36]);
	EXPECT_FLOAT_EQ(_input.distances[36], 0.f);

	VFHPlanner planner;
	const VFHPlanner::Result result = planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_TRUE(result.avoidance_required);
}

TEST_F(VFHObstacleAdapterTest, PlannerAvoidsConvertedForwardObstacle)
{
	_obstacle_map.distances[36] = 200;
	ASSERT_TRUE(convert());

	VFHPlanner planner;
	const VFHPlanner::Result result = planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_TRUE(result.avoidance_required);
	EXPECT_GT(fabsf(result.selected_direction), math::radians(20.f));
}

TEST_F(VFHObstacleAdapterTest, RejectsStaleAndFutureMessages)
{
	EXPECT_FALSE(VFHObstacleAdapter::convert(_obstacle_map, 1'200'001, 200'000, 0.f, 0.f, 0.f, _input));

	_obstacle_map.timestamp = 1'100'000;
	EXPECT_FALSE(convert());
}

TEST_F(VFHObstacleAdapterTest, AcceptsShortDropoutAndRecoversWithFreshFrame)
{
	constexpr uint64_t timeout = 500'000;

	ASSERT_TRUE(VFHObstacleAdapter::convert(_obstacle_map, 1'200'000, timeout, 0.f, 0.f, 0.f, _input));
	EXPECT_FALSE(VFHObstacleAdapter::convert(_obstacle_map, 1'400'001, timeout, 0.f, 0.f, 0.f, _input));

	_obstacle_map.timestamp = 1'500'000;
	EXPECT_TRUE(VFHObstacleAdapter::convert(_obstacle_map, 1'500'000, timeout, 0.f, 0.f, 0.f, _input));
}

TEST_F(VFHObstacleAdapterTest, RejectsUnsupportedFrameAndResolution)
{
	_obstacle_map.frame = obstacle_distance_s::MAV_FRAME_GLOBAL;
	EXPECT_FALSE(convert());

	_obstacle_map.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map.increment = 1.f;
	EXPECT_FALSE(convert());
}

TEST_F(VFHObstacleAdapterTest, RejectsAmbiguousMaximumDistance)
{
	_obstacle_map.max_distance = UINT16_MAX - 1;
	EXPECT_FALSE(convert());
}

TEST_F(VFHObstacleAdapterTest, DoesNotOverwriteOutputOnFailure)
{
	_input.target_direction = 0.75f;
	_obstacle_map.timestamp = 0;
	EXPECT_FALSE(convert());
	EXPECT_FLOAT_EQ(_input.target_direction, 0.75f);
}
