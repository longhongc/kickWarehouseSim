// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <memory>
#include "gtest/gtest.h"

#include "robot_manager/robot_manager.hpp"

TEST(DummyTests, dummy1)
{
  EXPECT_TRUE(true);
}

class TestRobotManager : public ::testing::Test
{
public:
  TestRobotManager() {}

  void SetUp() override
  {
    robot_manager = std::make_shared<RobotManager>();
    set_routine_client =
      robot_manager->create_client<SetRoutine>("set_routine");
  }

  std::shared_ptr<RobotManager> robot_manager;
  rclcpp::Client<SetRoutine>::SharedPtr set_routine_client;
};

TEST_F(TestRobotManager, testDefaultParams)
{
  auto origin_vec =
    robot_manager->get_parameter("origin").as_double_array();

  for (auto value : origin_vec) {
    EXPECT_FLOAT_EQ(value, 0);
  }

  auto waypoints_name =
    robot_manager->get_parameter("waypoints_name").as_string_array();

  EXPECT_EQ(static_cast<int>(waypoints_name.size()), 0);
}

TEST_F(TestRobotManager, testSetRoutineService)
{
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(robot_manager);
  auto start = robot_manager->now();
  while (rclcpp::ok() && (robot_manager->now() - start) < 1s) {
    executor.spin_some();
    if (!set_routine_client->wait_for_service(1s)) {
      RCLCPP_ERROR(
        robot_manager->get_logger(),
        "Set routine server not available after waiting 1s");

      FAIL();
    }
    rate.sleep();
  }
}
