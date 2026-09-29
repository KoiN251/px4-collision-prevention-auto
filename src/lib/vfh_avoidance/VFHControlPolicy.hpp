/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

class VFHControlPolicy
{
public:
	static constexpr int32_t MODE_STOP{0};
	static constexpr int32_t MODE_VFH_AVOID{1};
	// A missing path is tolerated briefly to avoid stopping on one bad update.
	static constexpr uint64_t NO_PATH_TIMEOUT_US{500'000};
	// Keep avoidance active until the target corridor has been clear for one second.
	static constexpr uint64_t CLEAR_TIMEOUT_US{1'000'000};

	struct Input {
		uint64_t now_us{0};
		// True when the planner found at least one valid free sector.
		bool path_available{false};
		// True when the mission direction is blocked or motion is unsafe.
		bool avoidance_required{false};
	};

	struct Output {
		// The caller may replace the normal XY velocity with the VFH command.
		bool vfh_velocity_override{false};
		// Persistent no-path state requires a zero-velocity response.
		bool no_path_stop{false};
		// Set only on the update that enters the no-path stop state.
		bool no_path_event{false};
	};

	static bool isVFHEnabled(int32_t auto_mode);

	Output update(int32_t auto_mode, const Input &input);
	void reset();

private:
	bool _avoidance_active{false};
	bool _no_path_stop{false};
	uint64_t _clear_start_us{0};
	uint64_t _no_path_start_us{0};
};
