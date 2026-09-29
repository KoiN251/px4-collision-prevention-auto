/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

class VFHPlanner
{
public:
	// PX4 obstacle_distance uses 72 sectors to represent a full 360-degree scan.
	static constexpr size_t BIN_COUNT{72};

	struct Config {
		// The vehicle and safety margin are combined to protect the complete footprint.
		float vehicle_radius{0.4f};
		float safety_margin{0.6f};
		float influence_distance{10.f};

		// Separate thresholds provide hysteresis between free and blocked states.
		float histogram_threshold_low{0.25f};
		float histogram_threshold_high{0.45f};

		// Limit the detour so the planner does not select a direction behind the target.
		float max_target_direction_change{1.5707963f};

		// Balance mission progress, current motion, and steering continuity.
		float target_weight{5.f};
		float current_weight{2.f};
		float previous_weight{2.f};
	};

	struct Input {
		// Distances are meters; angles are radians in one fixed navigation frame.
		float distances[BIN_COUNT] {};
		bool observed[BIN_COUNT] {};
		float angle_offset{0.f};
		float angle_increment{0.f};
		float target_direction{0.f};
		float current_direction{0.f};
	};

	struct Result {
		// Invalid means malformed input or that no safe sector was found.
		bool valid{false};
		bool avoidance_required{false};
		float selected_direction{0.f};
		float cost{0.f};
		uint8_t free_sector_count{0};
	};

	VFHPlanner() = default;
	explicit VFHPlanner(const Config &config) : _config(config) {}

	float getProtectedRadius() const { return _config.vehicle_radius + _config.safety_margin; }

	Result update(const Input &input);
	void reset();

private:
	static int wrapBin(int bin);
	static float angleDifference(float first, float second);
	int angleToBin(float angle, const Input &input) const;

	Config _config;
	bool _blocked[BIN_COUNT] {};
	bool _has_previous_direction{false};
	float _previous_direction{0.f};
};
