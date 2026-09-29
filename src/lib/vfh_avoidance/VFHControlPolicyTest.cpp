/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "VFHControlPolicy.hpp"

namespace
{

VFHControlPolicy::Input input(const uint64_t now_us, const bool path_available,
			      const bool avoidance_required)
{
	return {now_us, path_available, avoidance_required};
}

} // namespace

TEST(VFHControlPolicyTest, StopModeNeverOverridesVelocity)
{
	VFHControlPolicy policy;

	EXPECT_FALSE(VFHControlPolicy::isVFHEnabled(VFHControlPolicy::MODE_STOP));
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_STOP, input(1'000'000, true, true)).vfh_velocity_override);
}

TEST(VFHControlPolicyTest, UnknownModeFailsSafeToStop)
{
	VFHControlPolicy policy;

	EXPECT_FALSE(VFHControlPolicy::isVFHEnabled(-1));
	EXPECT_FALSE(VFHControlPolicy::isVFHEnabled(2));
	EXPECT_FALSE(policy.update(2, input(1'000'000, true, true)).vfh_velocity_override);
}

TEST(VFHControlPolicyTest, VFHOverridesOnlyForBlockedTargetWithValidPath)
{
	VFHControlPolicy policy;

	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(1'000'000, true, false)).vfh_velocity_override);
	EXPECT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					input(1'100'000, true, true)).vfh_velocity_override);
}

TEST(VFHControlPolicyTest, NoFreeSectorFallsBackBeforeTimeoutAndStopsAfterTimeout)
{
	VFHControlPolicy policy;

	ASSERT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					  input(1'000'000, true, true)).no_path_stop);
	ASSERT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(1'100'000, true, true)).vfh_velocity_override);

	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(2'000'000, false, false)).no_path_stop);
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(2'400'000, false, false)).no_path_stop);
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(2'500'000, false, false)).no_path_stop);
	const auto stop_output = policy.update(VFHControlPolicy::MODE_VFH_AVOID,
						input(2'500'001, false, false));
	EXPECT_TRUE(stop_output.no_path_stop);
	EXPECT_TRUE(stop_output.no_path_event);
	EXPECT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					input(2'500'001, false, false)).no_path_stop);
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(2'500'001, false, false)).no_path_event);
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
						 input(2'500'001, false, false)).vfh_velocity_override);

	const auto recovered_output = policy.update(VFHControlPolicy::MODE_VFH_AVOID,
									 input(2'600'001, true, true));
	EXPECT_FALSE(recovered_output.no_path_stop);
	EXPECT_TRUE(recovered_output.vfh_velocity_override);
}

TEST(VFHControlPolicyTest, ClearDirectionMustRemainStableForOneSecond)
{
	VFHControlPolicy policy;

	ASSERT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(1'000'000, true, true)).vfh_velocity_override);
	EXPECT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					input(1'500'000, true, false)).vfh_velocity_override);
	EXPECT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					input(2'000'000, true, false)).vfh_velocity_override);
	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(2'500'001, true, false)).vfh_velocity_override);
}

TEST(VFHControlPolicyTest, ResetAllowsVFHAfterSensorRecovery)
{
	VFHControlPolicy policy;

	ASSERT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(1'000'000, true, true)).vfh_velocity_override);
	policy.reset();

	EXPECT_FALSE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					 input(1'100'000, true, false)).vfh_velocity_override);
	EXPECT_TRUE(policy.update(VFHControlPolicy::MODE_VFH_AVOID,
					input(1'200'000, true, true)).vfh_velocity_override);
}
