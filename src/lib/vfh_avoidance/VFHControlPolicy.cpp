/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VFHControlPolicy.hpp"

bool VFHControlPolicy::isVFHEnabled(const int32_t auto_mode)
{
	return auto_mode == MODE_VFH_AVOID;
}

VFHControlPolicy::Output VFHControlPolicy::update(const int32_t auto_mode, const Input &input)
{
	Output output{};

	// Stop mode owns the legacy CP behavior and must not retain VFH state.
	if (!isVFHEnabled(auto_mode)) {
		reset();
		return output;
	}

	if (!input.path_available) {
		_clear_start_us = 0;

		// A transient no-path result is tolerated, but a persistent one latches a stop.
		if (_avoidance_active) {
			if (_no_path_start_us == 0) {
				_no_path_start_us = input.now_us;

			} else if (input.now_us - _no_path_start_us > NO_PATH_TIMEOUT_US) {
				_no_path_stop = true;
				_avoidance_active = false;
				output.no_path_event = true;
			}
		}

	} else {
		_no_path_start_us = 0;

		// A newly valid path clears the no-path latch before this cycle evaluates
		// whether avoidance should become active again.
		// A valid corridor means that the previous no-path condition has recovered.
		// Clear the stop latch before evaluating avoidance_required so VFH can resume
		// on this same update cycle.
		if (_no_path_stop) {
			_no_path_stop = false;
			_avoidance_active = false;
		}

		if (input.avoidance_required && !_no_path_stop && !_avoidance_active) {
			_avoidance_active = true;
			_clear_start_us = 0;

		} else if (_avoidance_active) {
			if (input.avoidance_required) {
				_clear_start_us = 0;

			} else if (_clear_start_us == 0) {
				_clear_start_us = input.now_us;

			} else if (input.now_us - _clear_start_us > CLEAR_TIMEOUT_US) {
				_avoidance_active = false;
				_clear_start_us = 0;
			}
		}
	}

	output.no_path_stop = _no_path_stop;
	output.vfh_velocity_override = input.path_available && _avoidance_active && !_no_path_stop;
	return output;
}

void VFHControlPolicy::reset()
{
	_avoidance_active = false;
	_no_path_stop = false;
	_clear_start_us = 0;
	_no_path_start_us = 0;
}
