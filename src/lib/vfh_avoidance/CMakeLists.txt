############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_library(VFHPlanner
	VFHPlanner.cpp
	VFHObstacleAdapter.cpp
	VFHVelocitySetpoint.cpp
	VFHDirectionalSpeedLimiter.cpp
	VFHControlPolicy.cpp
)

target_include_directories(VFHPlanner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(VFHPlanner PRIVATE mathlib)

px4_add_unit_gtest(SRC VFHPlannerTest.cpp LINKLIBS VFHPlanner)
px4_add_unit_gtest(SRC VFHArchitectureTest.cpp LINKLIBS VFHPlanner)
px4_add_unit_gtest(SRC VFHObstacleAdapterTest.cpp LINKLIBS VFHPlanner)
px4_add_unit_gtest(SRC VFHVelocitySetpointTest.cpp LINKLIBS VFHPlanner)
px4_add_unit_gtest(SRC VFHDirectionalSpeedLimiterTest.cpp LINKLIBS VFHPlanner)
px4_add_unit_gtest(SRC VFHControlPolicyTest.cpp LINKLIBS VFHPlanner)
