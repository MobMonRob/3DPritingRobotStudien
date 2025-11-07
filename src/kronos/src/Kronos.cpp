// Standard C++ headers
#include <memory>
#include <cstdlib>

// ROS 2 related headers
#include "rclcpp/rclcpp.hpp"

// MoveIt 2 related headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// ROS 2 Logging Macros
#define LOG_INFO(...) RCLCPP_INFO(LOGGER, __VA_ARGS__)
#define LOG_ERROR(...) RCLCPP_ERROR(LOGGER, __VA_ARGS__)

/**
 * @brief The main function for the UR3e Mover node.
 * * This function initializes ROS 2, sets up the Move Group Interface, and attempts 
 * to plan and execute a movement to a specified target pose.
 */
int main(int argc, char * argv[])
{
  // 1. Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // 2. Create a ROS 2 Node (This node is the client to the MoveIt 2 services)
  // We use the single-threaded executor for simplicity in this example.
  auto node = std::make_shared<rclcpp::Node>("kronos");

  // Spin the node asynchronously to handle callbacks from MoveIt 2 services
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Define the logger for cleaner output
  const rclcpp::Logger LOGGER = rclcpp::get_logger("mover_node");
  LOG_INFO("UR3e Mover Node initialized.");

  // 3. Define the Move Group Interface parameters
  const std::string PLANNING_GROUP = "ur_manipulator"; // Replace with your robot's planning group name
  
  // 4. Instantiate the Move Group Interface
  // The Move Group Interface is the primary class for high-level motion planning.
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 5. Configure Planning Parameters
  
  // Set the planning time limit (in seconds)
  move_group.setPlanningTime(10.0);
  
  // Set the number of attempts to plan
  move_group.setNumPlanningAttempts(5);
  
  // Set velocity scaling factor (0.0 to 1.0)
  move_group.setMaxVelocityScalingFactor(0.5);
  
  // Set acceleration scaling factor (0.0 to 1.0)
  move_group.setMaxAccelerationScalingFactor(0.5);

  LOG_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  LOG_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 6. Define the Target Pose (The goal position for the robot's end-effector)
  
  // Create a Pose message (geometry_msgs::msg::Pose)
  geometry_msgs::msg::Pose target_pose;
  
  // Position (in meters)
  target_pose.position.x = 0.20;  // 40 cm in front of the robot base
  target_pose.position.y = 0.00;  // Centered
  target_pose.position.z = 0.30;  // 40 cm above the robot base
  
  // Orientation (as a Quaternion - W, X, Y, Z). 
  // This example sets the orientation to be parallel to the Z-axis (pointing down/up).
  // A simple orientation (0, 0, 0, 1) means no rotation relative to the base frame.
  target_pose.orientation.w = 1.0;
  target_pose.orientation.x = 2.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  
  // 7. Set the target pose for the Move Group
  move_group.setPoseTarget(target_pose);
  LOG_INFO("Target pose set: x=%.2f, y=%.2f, z=%.2f", 
           target_pose.position.x, 
           target_pose.position.y, 
           target_pose.position.z);

  // 8. Plan the motion
  // moveit::planning_interface::MoveGroupInterface::Plan is a custom class 
  // that holds the result of a planning request.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) 
  {
    LOG_INFO("Planning successful! Attempting to execute the plan.");

    // 9. Execute the planned motion
    moveit::core::MoveItErrorCode execute_result = move_group.execute(my_plan);
    
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        LOG_INFO("Motion executed successfully!");
    } else {
        // --- FIX: Explicitly cast the enum class to an integer to satisfy the %d format specifier ---
        LOG_ERROR("Motion execution failed with error code: %d", *reinterpret_cast<int*>(&execute_result));
    }
  } 
  else 
  {
    LOG_ERROR("Planning failed. Check for collision or unreachable target.");
  }
  
  // 10. Shutdown and clean up
  rclcpp::shutdown();
  return 0;
}