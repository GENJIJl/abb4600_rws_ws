#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace abb_bringup
{
class TaskPlanningFixture : public testing::Test
{
public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, ControllerTopicsTest)
{
  // 等待其他节点完成启动。
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  auto topic_names_and_types = node_->get_topic_names_and_types();

  // 定义需要检查的话题及其消息类型。
  std::map<std::string, std::string> expected_topic_names_and_types = {
    { "/joint_trajectory_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory" },
    { "/joint_trajectory_controller/controller_state", "control_msgs/msg/JointTrajectoryControllerState" }
  };
  for (const auto& [topic_name, topic_type] : expected_topic_names_and_types)
  {
    auto it = topic_names_and_types.find(topic_name);

    // 检查话题是否存在；如果 find() 返回 map.end()，则测试失败。
    EXPECT_NE(it, topic_names_and_types.end());

    // 检查话题类型是否符合预期。
    EXPECT_EQ(it->second.front(), topic_type);
  }
}
}  // namespace abb_bringup

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
