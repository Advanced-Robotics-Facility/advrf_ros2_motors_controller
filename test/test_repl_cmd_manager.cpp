#include <gtest/gtest.h>
#include "advrf_ros2_motors_controller/repl_cmd_manager.hpp"

using namespace advrf_ros2_motors_controller;

class ReplManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("repl_manager_test");
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
  }
  void TearDown() override
  {
    executor->remove_node(node);
    executor.reset();
    node.reset();
    rclcpp::shutdown();
  }
  bool until(const std::function<bool()> &done)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!done() && std::chrono::steady_clock::now() < end)
      executor->spin_once(std::chrono::milliseconds(5));
    return done();
  }
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

TEST_F(ReplManagerTest, ArbitraryCommandsAndReentrantCallbacks)
{
  int calls = 0;
  auto service = node->create_service<ReplCmd>("generic_repl",
      [&calls](ReplCmd::Request::SharedPtr request, ReplCmd::Response::SharedPtr response)
      {
        ++calls;
        response->success = false; // Transport completion is distinct from command ACK.
        response->msg = std::to_string(request->request.type);
      });
  ReplCmdManager manager(*node, "generic_repl", 1.0);
  int replies = 0;
  ReplCmd::Request request;
  request.request.type = 1; // Not a motor START/STOP command.
  auto callback = [&replies](bool received, ReplCmd::Response::SharedPtr response)
  {
    EXPECT_TRUE(received);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->msg, "1");
    ++replies;
  };
  manager.send(nullptr, request, [&](bool received, ReplCmd::Response::SharedPtr response)
  {
    callback(received, response);
    manager.send(nullptr, request, callback);
  });
  manager.send(nullptr, request, callback);
  EXPECT_TRUE(until([&]() { return replies == 3; }));
  EXPECT_EQ(calls, 3);
  EXPECT_FALSE(manager.busy());
}

TEST_F(ReplManagerTest, UnavailableServiceCompletesOnceWithoutBlocking)
{
  ReplCmdManager manager(*node, "missing_repl", 0.05);
  int replies = 0;
  manager.send(nullptr, ReplCmd::Request{}, [&](bool received, ReplCmd::Response::SharedPtr response)
  {
    EXPECT_FALSE(received);
    EXPECT_EQ(response, nullptr);
    ++replies;
  });
  EXPECT_TRUE(until([&]() { return replies == 1; }));
  EXPECT_FALSE(manager.busy());
  EXPECT_THROW(manager.send(nullptr, ReplCmd::Request{}, {}), std::invalid_argument);
  EXPECT_THROW(ReplCmdManager(*node, "bad_timeout", 0), std::invalid_argument);
}
