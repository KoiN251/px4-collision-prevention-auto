/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <lib/mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

#include "VFHPlanner.hpp"

class VFHPlannerTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_input.angle_offset = -M_PI_F;
		_input.angle_increment = math::radians(5.f);
		_input.target_direction = 0.f;
		_input.current_direction = 0.f;

		for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
			_input.observed[bin] = true;
			_input.distances[bin] = 11.f;
		}
	}

	int binForAngle(float angle) const
	{
		int bin = static_cast<int>(roundf((angle - _input.angle_offset) / _input.angle_increment));
		return (bin % static_cast<int>(VFHPlanner::BIN_COUNT) + VFHPlanner::BIN_COUNT) % VFHPlanner::BIN_COUNT;
	}

	void setDistanceBetween(float minimum_angle, float maximum_angle, float distance)
	{
		for (float angle = minimum_angle; angle <= maximum_angle; angle += _input.angle_increment) {
			_input.distances[binForAngle(angle)] = distance;
		}
	}

	VFHPlanner _planner;
	VFHPlanner::Input _input{};
};

TEST_F(VFHPlannerTest, KeepsTargetDirectionWhenClear)
{
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_FALSE(result.avoidance_required);
	EXPECT_NEAR(result.selected_direction, 0.f, 1e-5f);
}

TEST_F(VFHPlannerTest, AvoidsObstacleInTargetDirection)
{
	_input.distances[binForAngle(0.f)] = 2.f;
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_TRUE(result.avoidance_required);
	EXPECT_GT(fabsf(result.selected_direction), math::radians(20.f));
}

TEST_F(VFHPlannerTest, SelectsOneSideOfWideForwardObstacle)
{
	setDistanceBetween(math::radians(-45.f), math::radians(45.f), 4.f);

	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_TRUE(result.avoidance_required);
	EXPECT_GT(fabsf(result.selected_direction), math::radians(45.f));
}

TEST_F(VFHPlannerTest, SideObstacleDoesNotBlockForwardTarget)
{
	_input.distances[binForAngle(math::radians(90.f))] = 2.f;
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_FALSE(result.avoidance_required);
	EXPECT_NEAR(result.selected_direction, 0.f, 1e-5f);
}

TEST_F(VFHPlannerTest, UserObstacleClusterLeavesForwardTargetClear)
{
	// Reproduce the reported scan. The close cluster is on the left, while the
	// mission target remains at 0 degrees.
	const float user_distances[] = {10.19f, 10.34f, 10.66f, 11.07f, 3.46f, 3.08f, 2.78f,
					2.84f, 3.13f, 3.50f, 4.09f, 4.79f, 5.99f, 8.00f};

	for (size_t offset = 0; offset < sizeof(user_distances) / sizeof(user_distances[0]); ++offset) {
		_input.distances[18 + offset] = user_distances[offset];
	}

	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_FALSE(result.avoidance_required);
	EXPECT_NEAR(result.selected_direction, 0.f, 1e-5f);
}

TEST_F(VFHPlannerTest, UnknownCoverageProducesNoPath)
{
	for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
		_input.observed[bin] = false;
	}

	const auto result = _planner.update(_input);
	EXPECT_FALSE(result.valid);
	EXPECT_EQ(result.free_sector_count, 0);
}

TEST_F(VFHPlannerTest, AllOccupiedSectorsProduceNoPath)
{
	for (float &distance : _input.distances) {
		distance = 1.f;
	}

	const auto result = _planner.update(_input);
	EXPECT_FALSE(result.valid);
	EXPECT_FALSE(result.avoidance_required);
	EXPECT_EQ(result.free_sector_count, 0);
}

TEST_F(VFHPlannerTest, RejectsOneDegreeInputWithoutAggregation)
{
	_input.angle_increment = math::radians(1.f);
	EXPECT_FALSE(_planner.update(_input).valid);
}

TEST_F(VFHPlannerTest, MissingTargetSectorBlocksMovement)
{
	_input.observed[binForAngle(0.f)] = false;
	const auto blocked_result = _planner.update(_input);
	ASSERT_TRUE(blocked_result.valid);
	EXPECT_TRUE(blocked_result.avoidance_required);

	_input.observed[binForAngle(0.f)] = true;
	EXPECT_FALSE(_planner.update(_input).avoidance_required);
}

TEST_F(VFHPlannerTest, ZeroDistanceBlocksTargetSector)
{
	_input.distances[binForAngle(0.f)] = 0.f;
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_TRUE(result.avoidance_required);
}

TEST_F(VFHPlannerTest, HistogramHysteresisRejectsThresholdNoise)
{
	_input.distances[binForAngle(0.f)] = 5.f;
	EXPECT_TRUE(_planner.update(_input).avoidance_required);

	_input.distances[binForAngle(0.f)] = 6.5f;
	EXPECT_TRUE(_planner.update(_input).avoidance_required);

	_input.distances[binForAngle(0.f)] = 8.f;
	EXPECT_FALSE(_planner.update(_input).avoidance_required);
}

TEST_F(VFHPlannerTest, RejectsGapTooNarrowForProtectedRadius)
{
	for (float &distance : _input.distances) {
		distance = 2.f;
	}

	setDistanceBetween(math::radians(-20.f), math::radians(20.f), 11.f);
	EXPECT_FALSE(_planner.update(_input).valid);
}

TEST_F(VFHPlannerTest, AcceptsGapWideEnoughForProtectedRadius)
{
	for (float &distance : _input.distances) {
		distance = 2.f;
	}

	setDistanceBetween(math::radians(-70.f), math::radians(70.f), 11.f);
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_NEAR(result.selected_direction, 0.f, 1e-5f);
}

TEST_F(VFHPlannerTest, StopsInUShapeInsteadOfSelectingReverseDirection)
{
	for (float &distance : _input.distances) {
		distance = 2.f;
	}

	setDistanceBetween(math::radians(140.f), math::radians(180.f), 11.f);
	setDistanceBetween(math::radians(-180.f), math::radians(-140.f), 11.f);
	EXPECT_FALSE(_planner.update(_input).valid);
}

TEST_F(VFHPlannerTest, HandlesAngleWrapAtPi)
{
	_input.target_direction = M_PI_F - math::radians(2.f);
	_input.current_direction = _input.target_direction;
	const auto result = _planner.update(_input);
	ASSERT_TRUE(result.valid);
	EXPECT_LT(fabsf(matrix::wrap_pi(result.selected_direction - _input.target_direction)), math::radians(3.f));
}

TEST_F(VFHPlannerTest, RejectsInvalidInput)
{
	_input.angle_increment = 0.f;
	EXPECT_FALSE(_planner.update(_input).valid);
}
