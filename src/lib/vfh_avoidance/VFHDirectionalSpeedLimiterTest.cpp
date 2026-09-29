/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHDirectionalSpeedLimiter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {
constexpr float PI{3.14159265358979323846f};
constexpr float DEG_TO_RAD{PI / 180.f};

VFHPlanner::Input freeScan()
{
	VFHPlanner::Input input{};

	for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
		input.distances[bin] = 10.f;
		input.observed[bin] = true;
	}

	input.angle_offset = -PI;
	input.angle_increment = 5.f * DEG_TO_RAD;
	return input;
}

VFHDirectionalSpeedLimiter::Output limit(const VFHPlanner::Input &input, const float selected_direction = 0.f)
{
	return VFHDirectionalSpeedLimiter::calculate(input, selected_direction, 3.f, 3.f, 5.66f, 0.4f,
			1.5f, 1.f, 3.f);
}
}

TEST(VFHDirectionalSpeedLimiterTest, FrontObstacleReducesSpeedBeforeStopThreshold)
{
	auto input = freeScan();
	input.distances[36] = 2.5f;
	const auto output = limit(input);

	ASSERT_TRUE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
	EXPECT_LT(output.speed_limit, 3.f);
	EXPECT_NEAR(output.corridor_distance, 2.5f, 0.01f);
}

TEST(VFHDirectionalSpeedLimiterTest, SideObstacleDoesNotReduceFreeFrontCorridor)
{
	auto input = freeScan();
	input.distances[54] = 1.f;
	const auto output = limit(input);

	ASSERT_TRUE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
	EXPECT_NEAR(output.speed_limit, 3.f, 0.01f);
}

TEST(VFHDirectionalSpeedLimiterTest, FreeRightCorridorKeepsSpeedWhenMissionDirectionIsBlocked)
{
	auto input = freeScan();
	input.distances[36] = 1.f;
	const auto output = limit(input, 45.f * DEG_TO_RAD);

	ASSERT_TRUE(output.valid);
	EXPECT_NEAR(output.speed_limit, 3.f, 0.01f);
}

TEST(VFHDirectionalSpeedLimiterTest, ObstacleInsideSelectedRightCorridorReducesSpeed)
{
	auto input = freeScan();
	input.distances[45] = 2.5f;
	const auto output = limit(input, 45.f * DEG_TO_RAD);

	ASSERT_TRUE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
	EXPECT_LT(output.speed_limit, 3.f);
}

TEST(VFHDirectionalSpeedLimiterTest, UnknownSelectedCorridorIsInvalid)
{
	auto input = freeScan();
	input.observed[36] = false;
	const auto output = limit(input);

	EXPECT_FALSE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
}

TEST(VFHDirectionalSpeedLimiterTest, LShapeSlowsNearCornerAndRecoversAfterCorner)
{
	auto input = freeScan();
	input.distances[45] = 8.f;
	const auto before_corner = limit(input, 45.f * DEG_TO_RAD);

	input.distances[45] = 2.5f;
	const auto near_corner = limit(input, 45.f * DEG_TO_RAD);

	input.distances[45] = 8.f;
	const auto after_corner = limit(input, 45.f * DEG_TO_RAD);

	EXPECT_NEAR(before_corner.speed_limit, 3.f, 0.01f);
	EXPECT_LT(near_corner.speed_limit, 3.f);
	EXPECT_NEAR(after_corner.speed_limit, 3.f, 0.01f);
}

TEST(VFHDirectionalSpeedLimiterTest, EmergencyStopStartsAtOnePointFiveMeters)
{
	auto input = freeScan();
	input.distances[36] = 1.5f;
	const auto output = limit(input);

	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(output.emergency_stop);
	EXPECT_FLOAT_EQ(output.speed_limit, 0.f);
}

TEST(VFHDirectionalSpeedLimiterTest, LoggedZeroPointSevenNineMetersIsAlreadyCritical)
{
	auto input = freeScan();
	input.distances[36] = 0.79f;
	const auto output = limit(input);

	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(output.emergency_stop);
	EXPECT_FLOAT_EQ(output.speed_limit, 0.f);
}

TEST(VFHDirectionalSpeedLimiterTest, InvalidBrakingInputFailsSafe)
{
	auto input = freeScan();
	const auto output = VFHDirectionalSpeedLimiter::calculate(input, 0.f, 3.f, 3.f, 0.f, 0.4f,
			1.5f, 1.f, 3.f);

	EXPECT_FALSE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
}

TEST(VFHDirectionalSpeedLimiterTest, SlowdownDistanceBelowStopDistanceIsClamped)
{
	auto input = freeScan();
	const auto output = VFHDirectionalSpeedLimiter::calculate(input, 0.f, 0.f, 3.f, 5.66f, 0.4f,
			1.5f, 1.f, 1.f);

	ASSERT_TRUE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
	EXPECT_NEAR(output.speed_limit, 3.f, 0.01f);
}

TEST(VFHDirectionalSpeedLimiterTest, DelayExhaustsBrakingDistanceAndCommandsZero)
{
	auto input = freeScan();
	input.distances[36] = 2.7f;
	const auto output = limit(input);

	ASSERT_TRUE(output.valid);
	EXPECT_FALSE(output.emergency_stop);
	EXPECT_FLOAT_EQ(output.speed_limit, 0.f);
}
