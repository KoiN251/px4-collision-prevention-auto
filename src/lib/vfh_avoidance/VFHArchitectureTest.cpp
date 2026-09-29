/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

#include <lib/mathlib/math/Functions.hpp>

#include "VFHPlanner.hpp"

namespace
{

VFHPlanner::Input clearInput()
{
	VFHPlanner::Input input{};
	input.angle_offset = -M_PI_F;
	input.angle_increment = math::radians(5.f);

	for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
		input.observed[bin] = true;
		input.distances[bin] = 11.f;
	}

	return input;
}

struct AsyncResult {
	uint64_t sensor_timestamp{0};
	uint64_t publication_timestamp{0};
	VFHPlanner::Result result{};

	bool isFresh(uint64_t now, uint64_t current_sensor_timestamp, uint64_t timeout) const
	{
		return result.valid && sensor_timestamp == current_sensor_timestamp
		       && publication_timestamp <= now && now - publication_timestamp <= timeout;
	}
};

} // namespace

TEST(VFHArchitectureTest, DirectCallUsesCurrentSensorFrame)
{
	VFHPlanner planner;
	const VFHPlanner::Input input = clearInput();
	const VFHPlanner::Result result = planner.update(input);

	ASSERT_TRUE(result.valid);
	EXPECT_NEAR(result.selected_direction, input.target_direction, 1e-5f);
}

TEST(VFHArchitectureTest, AsyncConsumerRejectsOldSensorFrame)
{
	VFHPlanner planner;
	AsyncResult output{1'000'000, 1'010'000, planner.update(clearInput())};

	EXPECT_TRUE(output.isFresh(1'050'000, 1'000'000, 100'000));
	EXPECT_FALSE(output.isFresh(1'050'000, 1'040'000, 100'000));
}

TEST(VFHArchitectureTest, AsyncConsumerRejectsLatePublication)
{
	VFHPlanner planner;
	AsyncResult output{1'000'000, 1'010'000, planner.update(clearInput())};

	EXPECT_FALSE(output.isFresh(1'120'001, 1'000'000, 100'000));
}

TEST(VFHArchitectureTest, PlannerRuntimeBenchmark)
{
	VFHPlanner planner;
	VFHPlanner::Input input = clearInput();
	constexpr int iteration_count = 100000;
	volatile float selected_direction_sum = 0.f;

	const auto start = std::chrono::steady_clock::now();

	for (int iteration = 0; iteration < iteration_count; ++iteration) {
		input.distances[36] = 2.f + static_cast<float>(iteration % 20) * 0.1f;
		selected_direction_sum += planner.update(input).selected_direction;
	}

	const auto elapsed = std::chrono::steady_clock::now() - start;
	const double average_microseconds = std::chrono::duration<double, std::micro>(elapsed).count() / iteration_count;
	std::cout << "[VFH benchmark] average planner time: " << average_microseconds << " us" << std::endl;
	EXPECT_TRUE(std::isfinite(selected_direction_sum));
}
