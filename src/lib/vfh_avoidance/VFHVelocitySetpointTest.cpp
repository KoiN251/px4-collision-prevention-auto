/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <lib/mathlib/math/Functions.hpp>

#include "VFHVelocitySetpoint.hpp"

TEST(VFHVelocitySetpointTest, ConvertsBodyDirectionToLocalNed)
{
	VFHPlanner::Result result{};
	result.valid = true;
	result.avoidance_required = true;
	result.selected_direction = math::radians(30.f);
	matrix::Vector2f velocity_setpoint{};

	ASSERT_TRUE(VFHVelocitySetpoint::generate(result, math::radians(60.f), 2.f, velocity_setpoint));
	EXPECT_NEAR(velocity_setpoint(0), 0.f, 1e-5f);
	EXPECT_NEAR(velocity_setpoint(1), 2.f, 1e-5f);
}

TEST(VFHVelocitySetpointTest, PreservesRequestedSpeed)
{
	VFHPlanner::Result result{};
	result.valid = true;
	result.avoidance_required = true;
	result.selected_direction = math::radians(-45.f);
	matrix::Vector2f velocity_setpoint{};

	ASSERT_TRUE(VFHVelocitySetpoint::generate(result, math::radians(10.f), 3.f, velocity_setpoint));
	EXPECT_NEAR(velocity_setpoint.norm(), 3.f, 1e-5f);
}

TEST(VFHVelocitySetpointTest, GeneratesClearTargetDirectionForActiveAvoidanceHysteresis)
{
	VFHPlanner::Result result{};
	result.valid = true;
	result.avoidance_required = false;
	result.selected_direction = math::radians(15.f);
	matrix::Vector2f velocity_setpoint{};

	ASSERT_TRUE(VFHVelocitySetpoint::generate(result, 0.f, 2.f, velocity_setpoint));
	EXPECT_NEAR(velocity_setpoint.norm(), 2.f, 1e-5f);
	EXPECT_NEAR(atan2f(velocity_setpoint(1), velocity_setpoint(0)), result.selected_direction, 1e-5f);
}

TEST(VFHVelocitySetpointTest, RejectsInvalidPlannerResultAndSpeed)
{
	VFHPlanner::Result result{};
	matrix::Vector2f velocity_setpoint{};

	EXPECT_FALSE(VFHVelocitySetpoint::generate(result, 0.f, 2.f, velocity_setpoint));
	result.valid = true;
	result.avoidance_required = true;
	EXPECT_FALSE(VFHVelocitySetpoint::generate(result, 0.f, 0.f, velocity_setpoint));
	EXPECT_FALSE(VFHVelocitySetpoint::generate(result, 0.f, NAN, velocity_setpoint));
}
