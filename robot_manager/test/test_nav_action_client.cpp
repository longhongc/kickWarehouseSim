// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <memory>

#include "gtest/gtest.h"

#include "robot_manager/nav_action_client.hpp"


TEST(DummyTests, dummyNavActionClient)
{
  EXPECT_TRUE(true);
}

class TestNavActionClient : public ::testing::Test
{
public:
  TestNavActionClient() {}

  void SetUp() override
  {
    test_node = rclcpp::Node::make_shared("test_nav_action_client_node");

    nav_to_pose_client = std::make_shared<NavActionClient>(test_node);
  }

  rclcpp::Node::SharedPtr test_node;
  std::shared_ptr<NavActionClient> nav_to_pose_client;
};

TEST_F(TestNavActionClient, testOnTask)
{
  EXPECT_FALSE(nav_to_pose_client->onTask());
}

TEST_F(TestNavActionClient, testSendGoal)
{
  auto origin = geometry_msgs::msg::PoseStamped();
  EXPECT_FALSE(nav_to_pose_client->sendGoal(origin));
}

TEST_F(TestNavActionClient, testGetResult)
{
  EXPECT_FALSE(nav_to_pose_client->getResult());
}

TEST_F(TestNavActionClient, testInitializeReset)
{
  nav_to_pose_client->reset();
  EXPECT_TRUE(nav_to_pose_client->initialize());
}

TEST_F(TestNavActionClient, testSetState)
{
  EXPECT_EQ(nav_to_pose_client->setState(ClientState::FINISH), ClientState::FINISH);
  EXPECT_TRUE(nav_to_pose_client->getResult());

  EXPECT_EQ(nav_to_pose_client->setState(ClientState::UNSET), ClientState::UNSET);
  EXPECT_TRUE(nav_to_pose_client->getResult());

  EXPECT_EQ(nav_to_pose_client->setState(ClientState::ON_TASK), ClientState::ON_TASK);
  EXPECT_FALSE(nav_to_pose_client->getResult());
}
