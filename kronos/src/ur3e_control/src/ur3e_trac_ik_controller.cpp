/**
 * UR3e controller using TRAC-IK for inverse kinematics.
 *
 * Solves IK for a sequence of Cartesian waypoints with TRAC-IK,
 * then executes the resulting joint trajectory via MoveIt's
 * MoveGroupInterface.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <thread>

static const std::string PLANNING_GROUP = "ur_manipulator";
static const std::string BASE_LINK      = "base_link";
static const std::string TIP_LINK       = "tool0";
static constexpr double  IK_TIMEOUT     = 0.05;   // seconds per solve
static constexpr double  IK_EPSILON     = 1e-5;

struct Waypoint {
  double x, y, z;
  double roll, pitch, yaw;   // orientation (radians)
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_trac_ik_controller");
  auto logger = node->get_logger();

  // Spin the node in a background thread so subscription callbacks
  // (e.g. joint_states for MoveIt's CurrentStateMonitor) get processed.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // --- Switch from scaled_joint_trajectory_controller to joint_trajectory_controller.
  //     The UR scaled controller enforces strict path tolerances that can
  //     cause PATH_TOLERANCE_VIOLATED aborts with fake hardware.
  RCLCPP_INFO(logger, "Switching to joint_trajectory_controller ...");
  {
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(logger, "controller_manager not available — skipping switch");
    } else {
      auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      req->activate_controllers   = {"joint_trajectory_controller"};
      req->deactivate_controllers = {"scaled_joint_trajectory_controller"};
      req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      auto future = client->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        RCLCPP_INFO(logger, "Controller switch OK.");
      } else {
        RCLCPP_WARN(logger, "Controller switch timed out — continuing anyway.");
      }
    }
  }

  // --- wait for controllers -----------------
  RCLCPP_INFO(logger, "Waiting for controllers to be ready ...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // --- MoveIt setup -----------------------------------------------------
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(logger, "Planning frame : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector   : %s", move_group.getEndEffectorLink().c_str());

  // --- Retrieve URDF from parameter server (for logging) ----------------
  std::string urdf_string;
  if (!node->get_parameter("robot_description", urdf_string)) {
    RCLCPP_ERROR(logger, "Failed to get 'robot_description' parameter.");
    rclcpp::shutdown();
    return 1;
  }

  // --- Initialise TRAC-IK solver ----------------------------------------
  // ROS2 rolling API: node reads URDF from the "robot_description" parameter
  TRAC_IK::TRAC_IK ik_solver(
      node, BASE_LINK, TIP_LINK, "robot_description",
      IK_TIMEOUT, IK_EPSILON, TRAC_IK::Speed);

  KDL::Chain chain;
  if (!ik_solver.getKDLChain(chain)) {
    RCLCPP_ERROR(logger, "TRAC-IK could not build KDL chain %s -> %s",
                 BASE_LINK.c_str(), TIP_LINK.c_str());
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  const unsigned int n_joints = chain.getNrOfJoints();
  RCLCPP_INFO(logger, "TRAC-IK ready  : %u joints (%s -> %s)",
              n_joints, BASE_LINK.c_str(), TIP_LINK.c_str());

  // --- Get current joint state as seed ----------------------------------
  // Wait until the joint-state monitor has received at least one message.
  std::vector<double> current_joints;
  for (int attempt = 0; attempt < 20; ++attempt) {
    current_joints = move_group.getCurrentJointValues();
    if (current_joints.size() == n_joints) break;
    RCLCPP_INFO(logger, "Waiting for joint states ... (%d)", attempt + 1);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_joints.size() != n_joints) {
    RCLCPP_ERROR(logger, "Could not get current joint state (got %zu, need %u). "
                 "Is the simulation running?",
                 current_joints.size(), n_joints);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  KDL::JntArray seed(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
    seed(i) = current_joints[i];

  // === Define waypoints =================================================
  // Orientation: tool pointing straight down (rx=π, ry=0, rz=0)
  std::vector<Waypoint> waypoints = {
    { 0.20, -0.20,  0.30,  M_PI, 0.0, 0.0},
    { 0.20,  0.20,  0.30,  M_PI, 0.0, 0.0},
    { 0.30,  0.00,  0.25,  M_PI, 0.0, 0.0},
    { 0.20, -0.20,  0.30,  M_PI, 0.0, 0.0},
  };

  RCLCPP_INFO(logger, "Solving IK for %zu waypoints with TRAC-IK ...",
              waypoints.size());

  // --- Solve IK for every waypoint --------------------------------------
  std::vector<std::vector<double>> joint_targets;
  joint_targets.reserve(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& wp = waypoints[i];

    // Build KDL target frame
    KDL::Rotation rot = KDL::Rotation::RPY(wp.roll, wp.pitch, wp.yaw);
    KDL::Frame    target(rot, KDL::Vector(wp.x, wp.y, wp.z));

    KDL::JntArray result(n_joints);
    int rc = ik_solver.CartToJnt(seed, target, result);

    if (rc < 0) {
      RCLCPP_ERROR(logger,
        "  Waypoint %zu (%.3f, %.3f, %.3f): IK failed (code %d)",
        i, wp.x, wp.y, wp.z, rc);
      RCLCPP_INFO(logger, "Done (aborted).");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    std::vector<double> jvals(n_joints);
    for (unsigned int j = 0; j < n_joints; ++j)
      jvals[j] = result(j);

    joint_targets.push_back(jvals);

    RCLCPP_INFO(logger,
      "  Waypoint %zu (%.3f, %.3f, %.3f): IK solved  [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      i, wp.x, wp.y, wp.z,
      jvals[0], jvals[1], jvals[2], jvals[3], jvals[4], jvals[5]);

    // Use this solution as seed for next waypoint (continuity)
    seed = result;
  }

  // --- Execute each joint target sequentially via MoveIt ----------------
  RCLCPP_INFO(logger, "Executing %zu waypoints ...", joint_targets.size());

  for (size_t i = 0; i < joint_targets.size(); ++i) {
    RCLCPP_INFO(logger, "  Moving to waypoint %zu ...", i);

    move_group.setJointValueTarget(joint_targets[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = move_group.plan(plan);
    if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "  Planning failed for waypoint %zu", i);
      break;
    }

    auto exec = move_group.execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "  Execution failed for waypoint %zu", i);
      break;
    }

    RCLCPP_INFO(logger, "  Reached waypoint %zu.", i);
  }

  RCLCPP_INFO(logger, "Done.");
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
