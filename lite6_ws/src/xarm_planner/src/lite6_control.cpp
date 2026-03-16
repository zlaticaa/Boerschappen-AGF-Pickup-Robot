#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
  fprintf(stderr, "[lite6_control] Ctrl-C caught, exit process...\n");
  exit(-1);
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lite6_control");
  RCLCPP_INFO(node->get_logger(), "lite6_control start");

  signal(SIGINT, exit_sig_handler);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("lite6_control");

  std::string group_name = "lite6";

  // Log
  RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());
  RCLCPP_INFO(node->get_logger(), "Initialising planner");

  xarm_planner::XArmPlanner planner(node, group_name);

  RCLCPP_INFO(node->get_logger(), "Setting target poses");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.2;
  target_pose1.orientation.x = 1;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 0;

  RCLCPP_INFO(node->get_logger(), "Starting planning!");
  // while (rclcpp::ok())
  // {
    // Plan and execute new position
    planner.planPoseTarget(target_pose1);
    planner.executePath();
  // }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}