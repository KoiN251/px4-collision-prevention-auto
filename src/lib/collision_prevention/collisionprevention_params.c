/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file collisionprevention_params.c
 *
 * Parameters defined by the collisionprevention lib.
 *
 * @author Tanja Baumann <tanja@auterion.com>
 */

/**
 * Minimum distance the vehicle should keep to all obstacles
 *
 * Only used in Position mode. Collision avoidance is disabled by setting this parameter to a negative value
 *
 * @min -1
 * @max 15
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DIST, -1.0f);

/**
 * Average delay of the range sensor message plus the tracking delay of the position controller in seconds
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 1
 * @unit s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DELAY, 0.4f);

/**
 * Angle left/right from the commanded setpoint by which the collision prevention algorithm can choose to change the setpoint direction
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 90
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_GUIDE_ANG, 30.f);

/**
 * Boolean to allow moving into directions where there is no sensor data (outside FOV)
 *
 * Only used in Position mode.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_GO_NO_DATA, 0);

/**
 * Enable collision safety in autonomous flight
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_AUTO_EN, 0);

/**
 * Minimum distance the vehicle should keep to obstacles in autonomous flight
 *
 * This is separate from CP_DIST, which remains the minimum distance used by
 * manual collision prevention.
 *
 * @min 0.1
 * @max 15
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_AUTO_DIST, 6.0f);

/**
 * Collision prevention behavior in autonomous flight
 *
 * Stop keeps the existing safe-velocity braking behavior. VFH Avoid selects a
 * free horizontal direction when the mission direction is blocked.
 *
 * @value 0 Stop
 * @value 1 VFH Avoid
 * @min 0
 * @max 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_AUTO_MODE, 0);

/**
 * Maximum horizontal velocity while autonomous VFH avoidance is active
 *
 * The mission cruise speed and MPC horizontal velocity limit are still
 * respected; this parameter adds a dedicated upper bound for VFH motion.
 *
 * @min 0.1
 * @max 12
 * @unit m/s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_VFH_MAX_VEL, 3.0f);

/**
 * Emergency stop distance for autonomous VFH avoidance
 *
 * VFH starts reducing speed before this threshold using the selected corridor.
 * When the projected obstacle distance reaches this threshold, Auto commands
 * an emergency stop instead of continuing the avoidance velocity.
 *
 * @min 0.1
 * @max 15
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_VFH_STOP_DIST, 1.5f);

/**
 * Maximum tilt angle when autonomous collision prevention is active
 *
 * Limits the horizontal acceleration requested by autonomous collision prevention
 * to reduce vehicle tilt while using obstacle sensors.
 *
 * @min 0
 * @max 45
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_MAX_ANGLE, 10.0f);

/**
 * Vehicle XY velocity threshold for collision stop detection
 *
 * @min 0.05
 * @max 1
 * @unit m/s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_STOP_VEL, 0.2f);

/**
 * Maximum tilt angle used for autonomous collision braking
 *
 * Allows stronger braking than normal autonomous collision prevention tilt.
 *
 * @min 0
 * @max 45
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_BRAKE_ANGLE, 30.0f);
