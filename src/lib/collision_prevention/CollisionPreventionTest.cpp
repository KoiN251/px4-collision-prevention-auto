/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "CollisionPrevention.hpp"
#include <mathlib/math/TrajMath.hpp>

using namespace matrix;

// to run: make tests TESTFILTER=CollisionPrevention
hrt_abstime mocked_time = 0;
const uint bin_size = CollisionPrevention::BIN_SIZE;
const uint bin_count = CollisionPrevention::BIN_COUNT;

class CollisionPreventionTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};

class TestCollisionPrevention : public CollisionPrevention
{
public:
	TestCollisionPrevention() : CollisionPrevention(nullptr) {}
	void paramsChanged() {CollisionPrevention::updateParamsImpl();}
	obstacle_distance_s &getObstacleMap() {return _obstacle_map_body_frame;}
	void test_addDistanceSensorData(distance_sensor_s &distance_sensor, const Quatf &attitude)
	{
		_addDistanceSensorData(distance_sensor, attitude);
	}
	void test_addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw)
	{
		_addObstacleSensorData(obstacle, vehicle_yaw, Quatf(Eulerf(0.f, 0.f, vehicle_yaw)));
	}
	void test_addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw,
					const Quatf &vehicle_attitude)
	{
		_addObstacleSensorData(obstacle, vehicle_yaw, vehicle_attitude);
	}
	void test_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index,
					 float vehicle_yaw_angle_rad)
	{
		_adaptSetpointDirection(setpoint_dir, setpoint_index, vehicle_yaw_angle_rad);
	}
	bool test_enterData(int map_index, float sensor_range, float sensor_reading)
	{
		return _enterData(map_index, sensor_range, sensor_reading);
	}
	void test_setObstacleBin(int bin, uint16_t distance_cm, uint16_t max_range_cm, hrt_abstime timestamp)
	{
		_obstacle_map_body_frame.timestamp = timestamp;
		_obstacle_map_body_frame.min_distance = 100;
		_obstacle_map_body_frame.max_distance = max_range_cm;
		_obstacle_map_body_frame.distances[bin] = distance_cm;
		_data_timestamps[bin] = timestamp;
		_data_maxranges[bin] = max_range_cm;
		_data_fov[bin] = true;
		_updateObstacleData();
	}
};

class TestTimingCollisionPrevention : public TestCollisionPrevention
{
public:
	TestTimingCollisionPrevention() : TestCollisionPrevention() {}
protected:
	hrt_abstime getTime() override
	{
		return mocked_time;
	}

	hrt_abstime getElapsedTime(const hrt_abstime *ptr) override
	{
		return mocked_time - *ptr;
	}
};

TEST_F(CollisionPreventionTest, instantiation) { CollisionPrevention cp(nullptr); }

TEST_F(CollisionPreventionTest, behaviorOff)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;

	// THEN: the collision prevention should be turned off by default
	EXPECT_FALSE(cp.is_active());
}

TEST_F(CollisionPreventionTest, noSensorData)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);

	// WHEN: we set the parameter check then apply the setpoint modification
	float value = 10; // try to keep 10m away from obstacles
	param_set(param, &value);
	cp.paramsChanged();

	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: collision prevention should be enabled and limit the speed to zero
	EXPECT_TRUE(cp.is_active());
	EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm());
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithObstacleMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint1(10, 0);
	Vector2f original_setpoint2(-10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param1 = param_handle(px4::params::CP_DIST);
	float value1 = 10; // try to keep 10m distance
	param_set(param1, &value1);
	param_t param2 = param_handle(px4::params::CP_GUIDE_ANG);
	float value2 = 0; // dont guide sideways
	param_set(param2, &value2);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			message.distances[i] = 101;

		} else {
			message.distances[i] = 10001;
		}
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);
	Vector2f modified_setpoint1 = original_setpoint1;
	Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, curr_vel);
	cp.modifySetpoint(modified_setpoint2, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the acceleration setpoint should be negative as its pushing you away from the obstacle, and sideways acceleration should be low
	// case 2: the acceleration setpoint should be lower
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);
	EXPECT_GT(0.f, modified_setpoint1(0)) << modified_setpoint1(0);
	EXPECT_EQ(0.f, fabsf(modified_setpoint1(1))) << modified_setpoint1(1);
	EXPECT_GT(0.f, modified_setpoint2(0))  << original_setpoint2(0);
	EXPECT_EQ(0.f, fabsf(modified_setpoint2(1))) << modified_setpoint2(1);
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithDistanceMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint1(10, 0);
	Vector2f original_setpoint2(-10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	distance_sensor_s message;
	message.timestamp = hrt_absolute_time();
	message.min_distance = 1.f;
	message.max_distance = 100.f;
	message.current_distance = 1.1f;

	message.variance = 0.1f;
	message.signal_quality = 100;
	message.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	message.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	message.h_fov = math::radians(50.f);
	message.v_fov = math::radians(30.f);

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(distance_sensor), distance_sensor_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	//WHEN:  We run the setpoint modification
	Vector2f modified_setpoint1 = original_setpoint1;
	Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, curr_vel);
	cp.modifySetpoint(modified_setpoint2, curr_vel);
	orb_unadvertise(distance_sensor_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the acceleration setpoint should be negative as its pushing you away from the obstacle, and sideways acceleration should be low
	// case 2: the acceleration setpoint should be lower
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_GT(0.f, modified_setpoint1(0)) << modified_setpoint1(0);
	EXPECT_EQ(0.f, fabsf(modified_setpoint1(1))) << modified_setpoint1(1);
	EXPECT_GT(0.f, modified_setpoint2(0))  << original_setpoint2(0);
	EXPECT_EQ(0.f, fabsf(modified_setpoint2(1))) << modified_setpoint2(1);
}

TEST_F(CollisionPreventionTest, testPurgeOldData)
{
	// GIVEN: a simple setup condition
	TestTimingCollisionPrevention cp;
	hrt_abstime start_time = hrt_absolute_time();
	mocked_time = start_time;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = start_time;
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 1; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message, message_lost_data;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = start_time;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;
	message_lost_data = message;

	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			message.distances[i] = 10001;
			message_lost_data.distances[i] = UINT16_MAX;

		} else if (i > 15 && i < 18) {
			message.distances[i] = 10001;
			message_lost_data.distances[i] = 10001;

		} else {
			message.distances[i] = UINT16_MAX;
			message_lost_data.distances[i] = UINT16_MAX;
		}
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	for (int i = 0; i < 10; i++) {
		Vector2f modified_setpoint = original_setpoint;
		cp.modifySetpoint(modified_setpoint, curr_vel);

		mocked_time = mocked_time + 100000; //advance time by 0.1 seconds
		message_lost_data.timestamp = mocked_time;
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message_lost_data);

		//at iteration 8 change the CP_GO_NO_DATA to True
		if (i == 8) {
			param_t param_allow = param_handle(px4::params::CP_GO_NO_DATA);
			float value_allow = 1;
			param_set(param_allow, &value_allow);
			cp.paramsChanged();
		}

		if (i < 6) {
			// THEN: If the data is new enough, the velocity setpoint should stay the same as the input
			// Note: direction will change slightly due to guidance
			EXPECT_FLOAT_EQ(original_setpoint.norm(), modified_setpoint.norm());

		} else {
			// THEN: If the data is expired, the velocity setpoint should be cut down to zero because there is no data
			//(even if CP_GO_NO_DATA is set to true, because we once had data in those bins and now lost the sensor)
			EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm()) << modified_setpoint(0) << "," << modified_setpoint(1);
		}
	}

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
}

TEST_F(CollisionPreventionTest, testNoRangeData)
{
	// GIVEN: a simple setup condition
	TestTimingCollisionPrevention cp;
	hrt_abstime start_time = hrt_absolute_time();
	mocked_time = start_time;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = start_time;
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message without any obstacle
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = start_time;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 9000;
	}


	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	for (int i = 0; i < 10; i++) {
		Vector2f modified_setpoint = original_setpoint;
		cp.modifySetpoint(modified_setpoint, curr_vel);

		//advance time by 0.1 seconds but no new message comes in
		mocked_time = mocked_time + 100000;

		if (i < 5) {
			// THEN: If the data is new enough, the velocity setpoint should stay the same as the input
			// Note: direction will change slightly due to guidance
			EXPECT_FLOAT_EQ(original_setpoint.norm(), modified_setpoint.norm());

		} else {
			// THEN: If the data is expired, the velocity setpoint should be cut down to zero because there is no data
			EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm()) << modified_setpoint(0) << "," << modified_setpoint(1);
		}
	}

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
}

TEST_F(CollisionPreventionTest, noBias)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2; // try to keep 2m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 100;
	message.max_distance = 2000;
	message.timestamp = hrt_absolute_time();
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 700;
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: setpoint should go into the same direction as the stick input
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(0), modified_setpoint.normalized()(0));
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(1), modified_setpoint.normalized()(1));
}

TEST_F(CollisionPreventionTest, outsideFOV)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 5; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360.f / distances_array_size;

	//fov from 45deg to 225deg
	for (int i = 0; i < distances_array_size; i++) {
		float angle = i * message.increment;

		if (angle > 45.f && angle < 225.f) {
			message.distances[i] = 700;

		} else {
			message.distances[i] = UINT16_MAX;
		}
	}

	// WHEN: we publish the message and modify the setpoint for different demanded setpoints
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);

	for (int i = 0; i < distances_array_size; i++) {
		float angle_deg = (float)i * message.increment;
		float angle_rad = math::radians(angle_deg);
		Vector2f original_setpoint = {10.f * cosf(angle_rad), 10.f * sinf(angle_rad)};
		Vector2f modified_setpoint = original_setpoint;
		message.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
		cp.modifySetpoint(modified_setpoint, curr_vel);

		//THEN: if the resulting setpoint demands velocities bigger zero, it must lie inside the FOV
		float setpoint_length = modified_setpoint.norm();

		if (setpoint_length > 0.f) {
			Vector2f setpoint_dir = modified_setpoint / setpoint_length;
			float sp_angle_body_frame = atan2(setpoint_dir(1), setpoint_dir(0));
			float sp_angle_deg = math::degrees(wrap_2pi(sp_angle_body_frame));
			EXPECT_GE(sp_angle_deg, 45.f);
			EXPECT_LE(sp_angle_deg, 225.f);
		}
	}

	orb_unadvertise(obstacle_distance_pub);
}

TEST_F(CollisionPreventionTest, goNoData)
{
	// GIVEN: a simple setup condition with the initial state (no distance data)
	TestCollisionPrevention cp;
	Vector2f curr_vel(2, 0);

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360.f / distances_array_size;

	//fov from 0deg to 20deg
	for (int i = 0; i < distances_array_size; i++) {
		float angle = i * message.increment;

		if (angle > 0.f && angle < 40.f) {
			message.distances[i] = 1000;

		} else {
			message.distances[i] = UINT16_MAX;
		}
	}

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: a setpoint outside the field of view
	Vector2f original_setpoint = {-5, 0};
	Vector2f modified_setpoint = original_setpoint;

	//THEN: the modified setpoint should be zero acceleration
	cp.modifySetpoint(modified_setpoint, curr_vel);
	EXPECT_FLOAT_EQ(modified_setpoint.norm(), 0.f);

	//WHEN: we change the parameter CP_GO_NO_DATA to allow flying ouside the FOV
	param_t param_allow = param_handle(px4::params::CP_GO_NO_DATA);
	float value_allow = 1;
	param_set(param_allow, &value_allow);
	cp.paramsChanged();

	//THEN: When all bins contain UINT_16MAX the setpoint should be zero even if CP_GO_NO_DATA=1
	modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	EXPECT_FLOAT_EQ(modified_setpoint.norm(), 0.f);

	//THEN: As soon as the range data contains any valid number, flying outside the FOV is allowed
	message.timestamp = hrt_absolute_time();
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);

	modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	EXPECT_FLOAT_EQ(modified_setpoint.norm(), original_setpoint.norm());
	orb_unadvertise(obstacle_distance_pub);
}

TEST_F(CollisionPreventionTest, jerkLimit)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: distance set to 5m
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 5; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 100;
	message.max_distance = 2000;
	message.timestamp = hrt_absolute_time();
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 700;
	}

	// AND: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	Vector2f modified_setpoint_default_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_default_jerk, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// AND: we now set max jerk to 0.1
	param = param_handle(px4::params::MPC_JERK_MAX);
	value = 0.1; // 0.1 maximum jerk
	param_set(param, &value);
	cp.paramsChanged();

	// WHEN: we run the setpoint modification again
	Vector2f modified_setpoint_limited_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_limited_jerk, curr_vel);

	// THEN: the new setpoint should be much higher than the one with default jerk, as the rate of change in acceleration is more limmited
	EXPECT_GT(modified_setpoint_limited_jerk.norm(), modified_setpoint_default_jerk.norm());

}
TEST_F(CollisionPreventionTest, addOutOfRangeDistanceSensorData)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	// Distance is out of Range
	distance_sensor.current_distance = -1.f;
	distance_sensor.signal_quality = 0;

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 0) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], distance_sensor.max_distance * 100.f);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}
	}
}

TEST_F(CollisionPreventionTest, addDistanceSensorDataNarrow)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(0.1 * bin_size);

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	// WHEN the sensor has a very narrow field of view
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 0) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}
}
TEST_F(CollisionPreventionTest, addDistanceSensorDataSlightlyLarger)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(1.1 * bin_size);

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	// WHEN the sensor has a very narrow field of view
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the the bins corresponding to -5°, 0° and 5° should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 71 || i <= 1) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}
}

TEST_F(CollisionPreventionTest, addDistanceSensorData)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;

	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (uint32_t i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
	}

	//WHEN: we add distance sensor data to the right
	distance_sensor.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
	distance_sensor.h_fov = math::radians(19.99f);
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	uint fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start = (90 - fov) / bin_size;
	uint end = (90 + fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}

	//WHEN: we add additionally distance sensor data to the left
	distance_sensor.orientation = distance_sensor_s::ROTATION_LEFT_FACING;
	distance_sensor.h_fov = math::radians(50.f);
	distance_sensor.current_distance = 8.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start2 = (270 - fov) / bin_size;
	uint end2 = (270 + fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else if (i >= start2 && i <= end2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}

	//WHEN: we add additionally distance sensor data to the front
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(10.1f);
	distance_sensor.current_distance = 3.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start3 = (360 - fov) / bin_size;
	uint end3 = (fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500)  << i;

		} else if (i >= start2 && i <= end2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800) << i;

		} else if (i >= start3 || i <= end3) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 300) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}


}

TEST_F(CollisionPreventionTest, addObstacleSensorData_attitude)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 10-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	int start = 2;
	int end = 6;

	for (int i = start; i <= end ; i++) {
		obstacle_msg.distances[i] = 500;
	}



	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add distance sensor data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}


	//WHEN: we add obstacle distance sensor data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, M_PI_2);

	//THEN: the correct bins in the map should be filled
	int offset =  bin_count - 90 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle distance sensor data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, -M_PI_4);

	//THEN: the correct bins in the map should be filled
	offset =  45 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle distance sensor data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, M_PI);

	//THEN: the correct bins in the map should be filled
	offset =  180 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, addObstacleSensorData_offset_bodyframe)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD; // Body Frame
	obstacle_msg.increment = 6.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 363°-39° deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i <= 6 ; i++) { // 36° at 6° increment
		obstacle_msg.distances[i] = 500;
	}

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the the bins from 0 to 40 in map should be filled, which correspond to the angles from -2.5° to 42.5°
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		if ((i >= 0 && i <= 8) || i == CollisionPrevention::BIN_COUNT - 1) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add the same obstacle distance sensor data with an angle offset of 30.5°
	obstacle_msg.angle_offset = 30.5f;
	// This then means our obstacle is between 27.5° and 69.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 30° to 70° in map should be filled, which correspond to the angles from 27.5° to 72.5°
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 6 && i <= 14) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we increase the offset to -30.5°
	obstacle_msg.angle_offset = -30.5f;
	// This then means our obstacle is between 326.5° and 8.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 325° to 10° in map should be filled, which correspond to the angles from 322.5° to 12.5°

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 65 || i <= 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, addObstacleSensorData_bodyframe)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 10-30 deg body frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));
	int start = 2;
	int end = 6;

	for (int i = start; i <= end ; i++) {
		obstacle_msg.distances[i] = 500;
	}


	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add obstacle data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the correct bins in the map should be filled

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, M_PI_2);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, -M_PI_4);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, M_PI);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

}

TEST_F(CollisionPreventionTest, addObstacleSensorDataProjectsTiltBeforeSectoring)
{
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg{};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD;
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	for (uint16_t &distance : obstacle_msg.distances) {
		distance = UINT16_MAX;
	}

	// A forward ray remains in body sector 0 after a 45 degree yaw, while
	// its range is shortened by the 10 degree pitch projection.
	obstacle_msg.distances[0] = 1000;
	const float pitch = math::radians(10.f);
	const float yaw = math::radians(45.f);
	cp.test_addObstacleSensorData(obstacle_msg, yaw, Quatf(Eulerf(0.f, pitch, yaw)));
	EXPECT_EQ(cp.getObstacleMap().distances[0], 985);

	// A right-facing ray remains in body sector 18 after a roll, but is
	// projected to the horizontal plane before it is stored.
	TestCollisionPrevention rolled_cp;
	for (uint16_t &distance : obstacle_msg.distances) {
		distance = UINT16_MAX;
	}

	obstacle_msg.distances[18] = 1000;
	const float roll = math::radians(10.f);
	rolled_cp.test_addObstacleSensorData(obstacle_msg, 0.f, Quatf(Eulerf(roll, 0.f, 0.f)));
	EXPECT_EQ(rolled_cp.getObstacleMap().distances[18], 985);

	// Combined roll/pitch changes the horizontal bearing, so a 45 degree
	// lidar sector must be allowed to populate the neighboring body sector.
	TestCollisionPrevention remapped_cp;
	for (uint16_t &distance : obstacle_msg.distances) {
		distance = UINT16_MAX;
	}

	obstacle_msg.distances[9] = 1000;
	const float combined_roll = math::radians(20.f);
	const float combined_pitch = math::radians(10.f);
	remapped_cp.test_addObstacleSensorData(obstacle_msg, 0.f,
							Quatf(Eulerf(combined_roll, combined_pitch, 0.f)));
		EXPECT_NE(remapped_cp.getObstacleMap().distances[8], UINT16_MAX);
		EXPECT_NE(remapped_cp.getObstacleMap().distances[9], UINT16_MAX);
		EXPECT_LT(remapped_cp.getObstacleMap().distances[8], 1000);
		EXPECT_EQ(remapped_cp.getObstacleMap().distances[8], remapped_cp.getObstacleMap().distances[9]);

		for (int i = 0; i < CollisionPrevention::BIN_COUNT; i++) {
			if (i != 8 && i != 9) {
				EXPECT_EQ(remapped_cp.getObstacleMap().distances[i], UINT16_MAX) << i;
			}
		}

		// A sector crossing 0 degrees must update both neighboring bins.
		TestCollisionPrevention boundary_cp;
		for (uint16_t &distance : obstacle_msg.distances) {
			distance = UINT16_MAX;
		}

		obstacle_msg.angle_offset = -1.f;
		obstacle_msg.distances[0] = 500;
		boundary_cp.test_addObstacleSensorData(obstacle_msg, 0.f);
		EXPECT_EQ(boundary_cp.getObstacleMap().distances[71], 500);
		EXPECT_EQ(boundary_cp.getObstacleMap().distances[0], 500);
		for (int i = 1; i < CollisionPrevention::BIN_COUNT - 1; i++) {
			EXPECT_EQ(boundary_cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		// A clear reading must not overwrite a nearer reading in an overlapping
		// raw sector from the same lidar message.
		TestCollisionPrevention merge_cp;
		obstacle_msg.distances[1] = obstacle_msg.max_distance + 1;
		merge_cp.test_addObstacleSensorData(obstacle_msg, 0.f);
		EXPECT_EQ(merge_cp.getObstacleMap().distances[71], 500);
		EXPECT_EQ(merge_cp.getObstacleMap().distances[0], 500);
		EXPECT_EQ(merge_cp.getObstacleMap().distances[1], obstacle_msg.max_distance + 1);

		// Sentinel readings remain useful if the attitude is invalid. In-range
		// readings are intentionally skipped because their projected range is
		// undefined without a valid attitude.
		TestCollisionPrevention invalid_attitude_cp;
		for (uint16_t &distance : obstacle_msg.distances) {
			distance = UINT16_MAX;
		}

		obstacle_msg.angle_offset = 0.f;
		obstacle_msg.distances[0] = 0;
		invalid_attitude_cp.test_addObstacleSensorData(obstacle_msg, 0.f, Quatf(0.f, 0.f, 0.f, 0.f));
		EXPECT_EQ(invalid_attitude_cp.getObstacleMap().distances[0], 0);
	}

	TEST_F(CollisionPreventionTest, addObstacleSensorDataRejectsUnsupportedResolution)
	{
		TestCollisionPrevention cp;
		obstacle_distance_s obstacle_msg{};
		obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD;
		obstacle_msg.increment = 1.f;
		obstacle_msg.min_distance = 20;
		obstacle_msg.max_distance = 2000;
		obstacle_msg.distances[0] = 500;

		cp.test_addObstacleSensorData(obstacle_msg, 0.f);

		for (int i = 0; i < CollisionPrevention::BIN_COUNT; i++) {
			EXPECT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}


TEST_F(CollisionPreventionTest, addObstacleSensorData_resolution_offset)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 6.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 363°-39° deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i <= 6 ; i++) { // 36° at 6° increment
		obstacle_msg.distances[i] = 500;
	}

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the the bins from 0 to 40 in map should be filled, which correspond to the angles from -2.5° to 42.5°
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 0 && i <= 8) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add the same obstacle distance sensor data with an angle offset of 30.5°
	obstacle_msg.angle_offset = 30.5f;
	// This then means our obstacle is between 27.5° and 69.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 30° to 70° in map should be filled, which correspond to the angles from 27.5° to 72.5°
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 6 && i <= 14) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we increase the offset to -30.5°
	obstacle_msg.angle_offset = -30.5f;
	// This then means our obstacle is between 326.5° and 8.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 325° to 10° in map should be filled, which correspond to the angles from 322.5° to 12.5°

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 65 || i <= 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, adaptSetpointDirection_distinct_minimum)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	const float vehicle_yaw = 0.f;

	//obstacle at 0-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i <= 6 ; i++) {
		obstacle_msg.distances[i] = 500;
	}

	obstacle_msg.distances[2] = 1000;

	//define setpoint
	Vector2f setpoint_dir(1, 0);
	float sp_angle_body_frame = atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw;
	float sp_angle_with_offset_deg = wrap(math::degrees(sp_angle_body_frame) - cp.getObstacleMap().angle_offset,
					      0.f, 360.f);
	int sp_index = floor(sp_angle_with_offset_deg / cp.getObstacleMap().increment);

	//set parameter
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 3; // try to keep 10m away from obstacles
	param_set(param, &value);
	cp.paramsChanged();

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_yaw);
	cp.test_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw);

	//THEN: the setpoint direction should be modified correctly
	EXPECT_EQ(sp_index, 2);
	EXPECT_FLOAT_EQ(setpoint_dir(0), 0.98480773f);
	EXPECT_FLOAT_EQ(setpoint_dir(1), 0.17364818f);
}

TEST_F(CollisionPreventionTest, adaptSetpointDirection_flat_minimum)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	const float vehicle_yaw = 0.f;

	//obstacle at 0-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i < 7 ; i++) {
		obstacle_msg.distances[i] = 500;
	}

	obstacle_msg.distances[1] = 1000;
	obstacle_msg.distances[2] = 1000;
	obstacle_msg.distances[3] = 1000;

	//define setpoint
	Vector2f setpoint_dir(1, 0);
	float sp_angle_body_frame = atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw;
	float sp_angle_with_offset_deg = wrap(math::degrees(sp_angle_body_frame) - cp.getObstacleMap().angle_offset,
					      0.f, 360.f);
	int sp_index = floor(sp_angle_with_offset_deg / cp.getObstacleMap().increment);

	//set parameter
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 3; // try to keep 10m away from obstacles
	param_set(param, &value);
	cp.paramsChanged();

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_yaw);
	cp.test_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw);

	//THEN: the setpoint direction should be modified correctly
	EXPECT_EQ(sp_index, 2);
	EXPECT_FLOAT_EQ(setpoint_dir(0), 0.98480773f);
	EXPECT_FLOAT_EQ(setpoint_dir(1), 0.17364818f);
}

TEST_F(CollisionPreventionTest, overlappingSensors)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message for a short range and a long range sensor
	obstacle_distance_s short_range_msg, short_range_msg_no_obstacle, long_range_msg;
	memset(&short_range_msg, 0xDEAD, sizeof(short_range_msg));
	short_range_msg.frame = short_range_msg.MAV_FRAME_GLOBAL; //north aligned
	short_range_msg.angle_offset = 0;
	short_range_msg.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(short_range_msg.distances) / sizeof(short_range_msg.distances[0]);
	short_range_msg.increment = 360 / distances_array_size;
	long_range_msg = short_range_msg;
	long_range_msg.min_distance = 100;
	long_range_msg.max_distance = 1000;
	short_range_msg.min_distance = 20;
	short_range_msg.max_distance = 200;
	short_range_msg_no_obstacle = short_range_msg;


	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			short_range_msg_no_obstacle.distances[i] = 201;
			short_range_msg.distances[i] = 150;
			long_range_msg.distances[i] = 500;

		} else {
			short_range_msg_no_obstacle.distances[i] = UINT16_MAX;
			short_range_msg.distances[i] = UINT16_MAX;
			long_range_msg.distances[i] = UINT16_MAX;
		}
	}


	// CASE 1
	//WHEN: we publish the long range sensor message
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &long_range_msg);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the long range measurement
	EXPECT_EQ(500, cp.getObstacleMap().distances[2]);

	// CASE 2
	// WHEN: we publish the short range message followed by a long range message
	short_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &short_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);
	long_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the short range measurement
	EXPECT_EQ(150, cp.getObstacleMap().distances[2]);

	// CASE 3
	// WHEN: we publish the short range message with values out of range followed by a long range message
	short_range_msg_no_obstacle.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &short_range_msg_no_obstacle);
	cp.modifySetpoint(modified_setpoint, curr_vel);
	long_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the short range measurement
	EXPECT_EQ(500, cp.getObstacleMap().distances[2]);

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
}

TEST_F(CollisionPreventionTest, enterData)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform

	//THEN: just after initialization all bins are at UINT16_MAX and any data should be accepted
	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range

	//WHEN: we add distance sensor data to the right with a valid reading
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
	distance_sensor.h_fov = math::radians(19.99f);
	distance_sensor.current_distance = 5.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the internal map should contain the distance sensor readings
	for (int i = 16; i < 20; i++) {
		EXPECT_EQ(500, cp.getObstacleMap().distances[i]) << i;
	}

	//THEN: bins 8 & 9 contain valid readings
	// a valid reading should only be accepted from sensors with shorter or equal range
	// a out of range reading should only be accepted from sensors with the same range

	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_FALSE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range

	//WHEN: we add distance sensor data to the right with an out of range reading
	distance_sensor.current_distance = 21.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the internal map should contain the distance sensor readings
	for (int i = 16; i < 20; i++) {
		EXPECT_EQ(2000, cp.getObstacleMap().distances[i]) << i;
	}

	//THEN: bins 8 & 9 contain readings out of range
	// a reading in range will be accepted in any case
	// out of range readings will only be accepted from sensors with bigger or equal range

	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range
}
TEST_F(CollisionPreventionTest, safeVelocityUsesMeasuredVelocity)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2.f;
	param_set(param, &value);
	cp.paramsChanged();

	cp.test_setObstacleBin(0, 1000, 2000, mocked_time);

	const Vector2f travel_direction{1.f, 0.f};
	const Vector2f slow_velocity{1.f, 0.f};
	const Vector2f fast_velocity{5.f, 0.f};

	const auto slow = cp.calculateSafeVelocity(travel_direction, slow_velocity, 4.f, 2.f);
	const auto fast = cp.calculateSafeVelocity(travel_direction, fast_velocity, 4.f, 2.f);

	EXPECT_TRUE(slow.valid);
	EXPECT_TRUE(fast.valid);
	EXPECT_GT(slow.stopping_distance, fast.stopping_distance);
	EXPECT_GT(slow.safe_velocity, fast.safe_velocity);
}
TEST_F(CollisionPreventionTest, safeVelocityCriticalWhenStoppingDistanceNegative)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2.f;
	param_set(param, &value);
	cp.paramsChanged();

	cp.test_setObstacleBin(0, 250, 2000, mocked_time);

	const auto result = cp.calculateSafeVelocity({1.f, 0.f}, {2.f, 0.f}, 4.f, 2.f);

	EXPECT_TRUE(result.valid);
	EXPECT_TRUE(result.critical);
	EXPECT_FLOAT_EQ(result.safe_velocity, 0.f);
}
TEST_F(CollisionPreventionTest, safeVelocityIgnoresObstacleBehindTravelDirection)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2.f;
	param_set(param, &value);
	cp.paramsChanged();

	cp.test_setObstacleBin(36, 300, 2000, mocked_time); // 180 deg, behind forward travel

	const auto result = cp.calculateSafeVelocity({1.f, 0.f}, {1.f, 0.f}, 4.f, 2.f);

	EXPECT_FALSE(result.valid);
}

TEST_F(CollisionPreventionTest, safeVelocityUsesBrakingAcceleration)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2.f;
	param_set(param, &value);
	cp.paramsChanged();

	cp.test_setObstacleBin(0, 1000, 2000, mocked_time);

	const auto low_braking_accel = cp.calculateSafeVelocity({1.f, 0.f}, {2.f, 0.f}, 4.f, 1.5f);
	const auto high_braking_accel = cp.calculateSafeVelocity({1.f, 0.f}, {2.f, 0.f}, 4.f, 4.f);

	EXPECT_TRUE(low_braking_accel.valid);
	EXPECT_TRUE(high_braking_accel.valid);
	EXPECT_NE(low_braking_accel.safe_velocity, high_braking_accel.safe_velocity);
	EXPECT_FLOAT_EQ(low_braking_accel.safe_velocity,
			math::trajectory::computeMaxSpeedFromDistance(4.f, 1.5f, low_braking_accel.stopping_distance, 0.f));
	EXPECT_FLOAT_EQ(high_braking_accel.safe_velocity,
			math::trajectory::computeMaxSpeedFromDistance(4.f, 4.f, high_braking_accel.stopping_distance, 0.f));
}

TEST_F(CollisionPreventionTest, safeVelocityUsesCallerMinimumDistance)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t param = param_handle(px4::params::CP_DIST);
	float manual_distance = 1.5f;
	param_set(param, &manual_distance);
	cp.paramsChanged();

	cp.test_setObstacleBin(0, 250, 2000, mocked_time);

	const auto manual_result = cp.calculateSafeVelocity({1.f, 0.f}, {2.f, 0.f}, 4.f, 2.f);
	const auto auto_result = cp.calculateSafeVelocity({1.f, 0.f}, {2.f, 0.f}, 4.f, 2.f, 2.5f);

	EXPECT_TRUE(manual_result.valid);
	EXPECT_TRUE(auto_result.valid);
	EXPECT_FALSE(manual_result.critical);
	EXPECT_TRUE(auto_result.critical);
	EXPECT_GT(manual_result.safe_velocity, auto_result.safe_velocity);
}

TEST_F(CollisionPreventionTest, twoMeterObstacleUsesAutoStoppingDistance)
{
	TestTimingCollisionPrevention cp;
	mocked_time = 1_s;

	param_t distance_param = param_handle(px4::params::CP_DIST);
	float manual_distance = 1.5f;
	param_set(distance_param, &manual_distance);

	param_t delay_param = param_handle(px4::params::CP_DELAY);
	float delay = 0.4f;
	param_set(delay_param, &delay);
	cp.paramsChanged();

	// Use the user's relevant geometry: a 2 m obstacle at -60 degrees,
	// while the vehicle is travelling in the 0 degree direction.
	cp.getObstacleMap().angle_offset = -180.f;
	cp.test_setObstacleBin(24, 200, 3000, mocked_time);

	const auto manual_result = cp.calculateSafeVelocity({1.f, 0.f}, {0.5f, 0.f}, 4.f, 2.f, 1.5f);
	const auto auto_result = cp.calculateSafeVelocity({1.f, 0.f}, {0.5f, 0.f}, 4.f, 2.f, 2.5f);

	EXPECT_TRUE(manual_result.valid);
	EXPECT_TRUE(auto_result.valid);
	EXPECT_FALSE(manual_result.critical);
	EXPECT_TRUE(auto_result.critical);
}
