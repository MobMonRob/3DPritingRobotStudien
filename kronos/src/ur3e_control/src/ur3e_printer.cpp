/**
 * UR3e 3D-printer controller.
 *
 * Reads a toolpath JSON file produced by a slicer,
 * adds a print-bed collision object to the MoveIt planning scene,
 * plans Cartesian paths through batches of waypoints (with TRAC-IK as
 * the kinematics plugin for IK), and executes them in sequence.
 *
 * MoveIt handles collision avoidance automatically — any Cartesian
 * segment that would collide with the bed (or any other scene object)
 * will be rejected at planning time.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "json.hpp"

#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <thread>
#include <algorithm>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static const std::string PLANNING_GROUP = "ur_manipulator";

// Print-bed geometry (metres, relative to base_link)
static constexpr double BED_X      =  0.30;   // centre X
static constexpr double BED_Y      =  0.00;   // centre Y
static constexpr double BED_Z      =  0.095;  // centre Z (half-thickness below top surface at 0.10)
static constexpr double BED_SIZE_X =  0.30;   // length
static constexpr double BED_SIZE_Y =  0.30;   // width
static constexpr double BED_SIZE_Z =  0.01;   // thickness
static constexpr double DEFAULT_WORKPIECE_Z_OFFSET = 0.012;  // match legacy TCP clearance above the bed
static constexpr double PRINT_VISUAL_WIDTH = 0.003;

// Cartesian planning
static constexpr double CART_EEF_STEP    = 0.001;  // 1 mm interpolation step
static constexpr double CART_JUMP_THRESH = 0.0;    // disable jump check
static constexpr double MIN_COVERAGE     = 0.95;   // require 95 % path coverage
static constexpr size_t BATCH_SIZE       = 50;      // waypoints per Cartesian call
static constexpr size_t RECOVERY_LOOKAHEAD = 10;    // joint-space recovery probes

// Approach: lift above first waypoint, then descend in a straight line
static constexpr double APPROACH_HEIGHT  = 0.05;   // 50 mm above first point
static constexpr int    PLAN_ATTEMPTS    = 10;     // OMPL attempts (pick shortest)

// Velocity / acceleration scaling (applied to both joint-space and Cartesian moves)
static constexpr double VELOCITY_SCALING = 0.3;
static constexpr double ACCEL_SCALING    = 0.3;

// ---------------------------------------------------------------------------
// Toolpath parser
// ---------------------------------------------------------------------------
// Kronos JSON toolpath export with a top-level "waypoints" array.
// Positions are in millimetres; orientation is stored as a quaternion.
// ---------------------------------------------------------------------------
struct ToolPoint { double x, y, z, a, b, c; };
struct LoadedToolpath {
  std::vector<ToolPoint> points;
  std::string reference_frame;
};

struct PrintedSegment {
  size_t begin;
  size_t end;
};

using json = nlohmann::json;

// Returns the quaternion for a given toolpoint's RPY (A=roll, B=pitch, C=yaw).
static tf2::Quaternion tool_orientation(const ToolPoint& p) {
  tf2::Quaternion q;
  q.setRPY(p.a, p.b, p.c);
  return q;
}

// MEDUSA exports workpiece toolpaths in a Y-up basis (X/Z plane = bed,
// +Y = layer height). The robot plans in the usual ROS Z-up basis
// (X/Y plane = bed, +Z = layer height).
static tf2::Vector3 medusa_workpiece_position_to_robot_basis(
    double x, double y, double z) {
  return {x, -z, y};
}

static tf2::Vector3 medusa_tcp_direction_to_robot_basis(
    const tf2::Vector3& medusa_direction) {
  const tf2::Vector3 robot_direction = medusa_workpiece_position_to_robot_basis(
      medusa_direction.x(), medusa_direction.y(), medusa_direction.z());
  return -robot_direction;
}

static tf2::Quaternion quaternion_from_z_axis(const tf2::Vector3& target_direction) {
  tf2::Vector3 to = target_direction;
  if (to.length2() <= 1e-12) {
    return tf2::Quaternion::getIdentity();
  }
  to.normalize();

  const tf2::Vector3 from(0.0, 0.0, 1.0);
  const double dot = from.dot(to);
  if (dot < -1.0 + 1e-6) {
    return tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
  }

  const tf2::Vector3 cross = from.cross(to);
  const double w = 1.0 + dot;
  const double len = std::sqrt(w * w + cross.length2());
  if (len <= 1e-12) {
    return tf2::Quaternion::getIdentity();
  }

  tf2::Quaternion q(cross.x() / len, cross.y() / len, cross.z() / len, w / len);
  q.normalize();
  return q;
}

static LoadedToolpath parse_json(const std::string& path,
                                 double workpiece_origin_x,
                                 double workpiece_origin_y,
                                 double workpiece_origin_z) {
  LoadedToolpath toolpath;
  std::ifstream f(path);
  if (!f.is_open()) return toolpath;
  json doc;
  try {
    f >> doc;
  } catch (...) {
    return toolpath;
  }

  if (!doc.is_object() || !doc.contains("waypoints") || !doc["waypoints"].is_array()) {
    return toolpath;
  }

  toolpath.reference_frame = doc.value("reference_frame", "base_link");
  const bool is_workpiece_frame = toolpath.reference_frame == "workpiece";

  for (const auto& waypoint : doc["waypoints"]) {
    if (!waypoint.is_object()) {
      continue;
    }
    if (!waypoint.contains("x") || !waypoint.contains("y") || !waypoint.contains("z") ||
        !waypoint.contains("qw") || !waypoint.contains("qx") ||
        !waypoint.contains("qy") || !waypoint.contains("qz")) {
      continue;
    }

    try {
      tf2::Quaternion medusa_q(
          waypoint.at("qx").get<double>(),
          waypoint.at("qy").get<double>(),
          waypoint.at("qz").get<double>(),
          waypoint.at("qw").get<double>());
      medusa_q.normalize();

      double x = waypoint.at("x").get<double>() / 1000.0;
      double y = waypoint.at("y").get<double>() / 1000.0;
      double z = waypoint.at("z").get<double>() / 1000.0;

      tf2::Quaternion robot_q = medusa_q;
      if (is_workpiece_frame) {
        const tf2::Vector3 robot_position = medusa_workpiece_position_to_robot_basis(x, y, z);
        x = robot_position.x();
        y = robot_position.y();
        z = robot_position.z();

        const tf2::Vector3 medusa_tcp_direction = tf2::quatRotate(
            medusa_q, tf2::Vector3(0.0, 0.0, 1.0));
        const tf2::Vector3 robot_tcp_direction =
            medusa_tcp_direction_to_robot_basis(medusa_tcp_direction);
        robot_q = quaternion_from_z_axis(robot_tcp_direction);
      }

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(robot_q).getRPY(roll, pitch, yaw);

      if (is_workpiece_frame) {
        x += workpiece_origin_x;
        y += workpiece_origin_y;
        z += workpiece_origin_z;
      }

      toolpath.points.push_back({x, y, z, roll, pitch, yaw});
    } catch (...) {
      continue;
    }
  }
  return toolpath;
}

static LoadedToolpath load_toolpath(const std::string& path,
                                    double workpiece_origin_x,
                                    double workpiece_origin_y,
                                    double workpiece_origin_z) {
  return parse_json(path, workpiece_origin_x, workpiece_origin_y, workpiece_origin_z);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Remove 2π wraps from a trajectory so each joint stays continuous
// relative to the robot's actual starting position.
static void unwrap_trajectory(
    moveit_msgs::msg::RobotTrajectory& traj,
    const std::vector<double>& start_joints)
{
  auto& pts = traj.joint_trajectory.points;
  if (pts.empty()) return;

  // Use the actual current joint values as the reference, not the plan's
  // first point (which may already have a 2π offset).
  std::vector<double> prev = start_joints;

  for (auto& pt : pts) {
    for (size_t j = 0; j < pt.positions.size(); ++j) {
      double diff = pt.positions[j] - prev[j];
      if (diff > M_PI)       pt.positions[j] -= 2.0 * M_PI * std::ceil((diff - M_PI) / (2.0 * M_PI));
      else if (diff < -M_PI) pt.positions[j] += 2.0 * M_PI * std::ceil((-diff - M_PI) / (2.0 * M_PI));
      prev[j] = pt.positions[j];
    }
  }
}

// Apply time-optimal time parameterization to a Cartesian trajectory.
// computeCartesianPath() returns joint positions only (no velocities /
// accelerations and no valid time stamps), which causes controllers to abort.
// TOTG fills in the missing kinodynamic information.
static bool retime_trajectory(
    moveit_msgs::msg::RobotTrajectory& traj,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::RobotState& ref_state)
{
  robot_trajectory::RobotTrajectory rt(robot_model, PLANNING_GROUP);
  rt.setRobotTrajectoryMsg(ref_state, traj);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, VELOCITY_SCALING, ACCEL_SCALING))
    return false;
  rt.getRobotTrajectoryMsg(traj);
  return true;
}

static geometry_msgs::msg::Pose make_pose(double x, double y, double z,
                                           const tf2::Quaternion& q) {
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = tf2::toMsg(q);
  return p;
}

static bool plan_shortest_pose_target(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan& best_plan,
    double& best_len,
    int attempts = PLAN_ATTEMPTS)
{
  move_group.setPoseTarget(target_pose);
  move_group.setNumPlanningAttempts(attempts);

  best_len = std::numeric_limits<double>::max();
  bool found = false;

  for (int attempt = 0; attempt < attempts; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      continue;

    double len = 0.0;
    const auto& pts = plan.trajectory_.joint_trajectory.points;
    for (size_t i = 1; i < pts.size(); ++i) {
      double seg = 0.0;
      for (size_t j = 0; j < pts[i].positions.size(); ++j) {
        double d = pts[i].positions[j] - pts[i - 1].positions[j];
        seg += d * d;
      }
      len += std::sqrt(seg);
    }
    if (len < best_len) {
      best_len = len;
      best_plan = plan;
      found = true;
    }
  }

  move_group.setNumPlanningAttempts(1);
  return found;
}

static void add_print_bed(
    moveit::planning_interface::PlanningSceneInterface& psi,
  const std::string& frame,
  double bed_x,
  double bed_y,
  double bed_z) {
  moveit_msgs::msg::CollisionObject bed;
  bed.header.frame_id = frame;
  bed.id = "print_bed";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {BED_SIZE_X, BED_SIZE_Y, BED_SIZE_Z};

  geometry_msgs::msg::Pose bed_pose;
  bed_pose.position.x = bed_x;
  bed_pose.position.y = bed_y;
  bed_pose.position.z = bed_z;
  bed_pose.orientation.w = 1.0;

  bed.primitives.push_back(box);
  bed.primitive_poses.push_back(bed_pose);
  bed.operation = moveit_msgs::msg::CollisionObject::ADD;

  psi.applyCollisionObject(bed);
}

static void publish_printed_segments(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const std::string& frame,
    const std::vector<ToolPoint>& points,
    const std::vector<PrintedSegment>& segments)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 0;
  for (const auto& segment : segments) {
    if (segment.end <= segment.begin + 1 || segment.end > points.size()) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.ns = "printed_part";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = PRINT_VISUAL_WIDTH;
    marker.color.r = 1.0f;
    marker.color.g = 0.45f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;

    marker.points.reserve(segment.end - segment.begin);
    for (size_t i = segment.begin; i < segment.end; ++i) {
      geometry_msgs::msg::Point point;
      point.x = points[i].x;
      point.y = points[i].y;
      point.z = points[i].z;
      marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
  }

  publisher->publish(marker_array);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_printer");
  auto logger = node->get_logger();

  // Declare parameters
  node->declare_parameter<std::string>("toolpath_file", "");
  node->declare_parameter<double>("print_speed", 0.1);       // m/s TCP linear speed
  node->declare_parameter<double>("travel_speed", 0.3);      // m/s for non-print moves
  node->declare_parameter<double>("bed_x", BED_X);
  node->declare_parameter<double>("bed_y", BED_Y);
  node->declare_parameter<double>("bed_z", BED_Z);

  std::string toolpath_file = node->get_parameter("toolpath_file").as_string();
  double print_speed  = node->get_parameter("print_speed").as_double();
  double bed_x = node->get_parameter("bed_x").as_double();
  double bed_y = node->get_parameter("bed_y").as_double();
  double bed_z = node->get_parameter("bed_z").as_double();
  node->declare_parameter<double>("workpiece_z_offset", DEFAULT_WORKPIECE_Z_OFFSET);
  double workpiece_z_offset = node->get_parameter("workpiece_z_offset").as_double();
  node->declare_parameter<double>("workpiece_origin_x", bed_x);
  node->declare_parameter<double>("workpiece_origin_y", bed_y);
  node->declare_parameter<double>("workpiece_origin_z", bed_z + 0.5 * BED_SIZE_Z + workpiece_z_offset);
  double workpiece_origin_x = node->get_parameter("workpiece_origin_x").as_double();
  double workpiece_origin_y = node->get_parameter("workpiece_origin_y").as_double();
  double workpiece_origin_z = node->get_parameter("workpiece_origin_z").as_double();
  (void)print_speed;  // used via velocity scaling below

  // Background spinner for subscriptions
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // --- Wait for controllers (launched with joint_trajectory_controller) --
  RCLCPP_INFO(logger, "Waiting for controllers ...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // --- MoveIt setup -----------------------------------------------------
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(VELOCITY_SCALING);
  move_group.setMaxAccelerationScalingFactor(ACCEL_SCALING);

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  auto printed_part_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "printed_part", rclcpp::QoS(1).transient_local());
  std::vector<PrintedSegment> printed_segments;
    publish_printed_segments(
      printed_part_publisher,
      move_group.getPlanningFrame(),
      std::vector<ToolPoint>{},
      printed_segments);

  RCLCPP_INFO(logger, "Planning frame : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector   : %s", move_group.getEndEffectorLink().c_str());

  // --- Wait for joint states --------------------------------------------
  const size_t expected_joints = 6;
  std::vector<double> current_joints;
  for (int i = 0; i < 20; ++i) {
    current_joints = move_group.getCurrentJointValues();
    if (current_joints.size() == expected_joints) break;
    RCLCPP_INFO(logger, "Waiting for joint states ... (%d)", i + 1);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_joints.size() != expected_joints) {
    RCLCPP_ERROR(logger, "No joint states — is the simulation running?");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }

  // --- Add print bed to collision scene ---------------------------------
  RCLCPP_INFO(logger, "Adding print bed to planning scene ...");
  // add_print_bed(planning_scene, move_group.getPlanningFrame(), bed_x, bed_y, bed_z);
  rclcpp::sleep_for(std::chrono::seconds(1));   // let the scene propagate

  // --- Load toolpath ----------------------------------------------------
  if (toolpath_file.empty()) {
    RCLCPP_ERROR(logger, "No toolpath_file parameter set. "
                 "Use: ros2 launch ... toolpath_file:=/path/to/file.json");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  auto toolpath = load_toolpath(
      toolpath_file, workpiece_origin_x, workpiece_origin_y, workpiece_origin_z);
  if (toolpath.points.empty()) {
    RCLCPP_ERROR(logger, "Toolpath file '%s' is empty or unreadable.",
                 toolpath_file.c_str());
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(logger, "Loaded %zu waypoints from '%s' (reference frame: %s)",
              toolpath.points.size(), toolpath_file.c_str(), toolpath.reference_frame.c_str());
  if (toolpath.reference_frame == "workpiece") {
    RCLCPP_INFO(logger, "Resolved workpiece origin in %s: [%.3f, %.3f, %.3f] m",
                move_group.getPlanningFrame().c_str(),
                workpiece_origin_x, workpiece_origin_y, workpiece_origin_z);
  }

  // --- Move to approach pose above the first waypoint --------------------
  // Step 1: joint-space move to a point APPROACH_HEIGHT above waypoint 0.
  //         Multiple OMPL attempts → keep the shortest path.
  // Step 2: Cartesian straight-line descent to the actual first point.
  {
    auto approach_pose = make_pose(
      toolpath.points[0].x, toolpath.points[0].y,
      toolpath.points[0].z + APPROACH_HEIGHT, tool_orientation(toolpath.points[0]));

    move_group.setPoseTarget(approach_pose);

    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    double best_len = std::numeric_limits<double>::max();
    bool found = plan_shortest_pose_target(move_group, approach_pose, best_plan, best_len);

    if (!found) {
      RCLCPP_ERROR(logger, "Cannot plan to approach pose after %d attempts.",
                   PLAN_ATTEMPTS);
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(logger, "Best approach path length: %.3f rad (out of %d attempts)",
                best_len, PLAN_ATTEMPTS);

    unwrap_trajectory(best_plan.trajectory_, move_group.getCurrentJointValues());
    if (move_group.execute(best_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Failed to reach approach pose.");
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Step 2: straight-line descent to the actual first waypoint
    auto first_pose = make_pose(toolpath.points[0].x, toolpath.points[0].y,
                                toolpath.points[0].z, tool_orientation(toolpath.points[0]));
    std::vector<geometry_msgs::msg::Pose> descent{first_pose};
    moveit_msgs::msg::RobotTrajectory descent_traj;
    double cov = move_group.computeCartesianPath(
        descent, CART_EEF_STEP, CART_JUMP_THRESH, descent_traj);
    if (cov < 0.99) {
      RCLCPP_WARN(logger, "Cartesian descent to first waypoint failed "
                  "(coverage %.1f %%) — falling back to joint-space planning.",
                  cov * 100.0);
      moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
      double fallback_len = std::numeric_limits<double>::max();
      if (!plan_shortest_pose_target(move_group, first_pose, fallback_plan, fallback_len)) {
        RCLCPP_ERROR(logger, "Fallback planning to first waypoint failed.");
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
      }
      unwrap_trajectory(fallback_plan.trajectory_, move_group.getCurrentJointValues());
      if (move_group.execute(fallback_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to reach first waypoint with fallback planning.");
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
      }
      RCLCPP_INFO(logger, "Reached start position via joint-space fallback.");
    } else {
      if (!retime_trajectory(descent_traj, move_group.getRobotModel(),
                              *move_group.getCurrentState())) {
        RCLCPP_ERROR(logger, "Time parameterization of descent trajectory failed.");
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
      }
      unwrap_trajectory(descent_traj, move_group.getCurrentJointValues());
      if (move_group.execute(descent_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to descend to first waypoint.");
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
      }
      RCLCPP_INFO(logger, "Reached start position.");
    }
  }

  // --- Execute toolpath in Cartesian batches ----------------------------
  size_t total  = toolpath.points.size();
  size_t done   = 0;

  while (done < total) {
    size_t end = std::min(done + BATCH_SIZE, total);

    // Build batch of Cartesian poses
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(end - done);
    for (size_t i = done; i < end; ++i) {
      poses.push_back(make_pose(
          toolpath.points[i].x, toolpath.points[i].y, toolpath.points[i].z,
          tool_orientation(toolpath.points[i])));
    }

    // Plan Cartesian path (collision-aware through MoveIt planning scene)
    moveit_msgs::msg::RobotTrajectory trajectory;
    double coverage = move_group.computeCartesianPath(
        poses, CART_EEF_STEP, CART_JUMP_THRESH, trajectory);

    size_t planned_pts = static_cast<size_t>(
        std::round(coverage * static_cast<double>(poses.size())));

    RCLCPP_INFO(logger, "Batch [%zu–%zu] coverage: %.1f %% (%zu/%zu pts)",
                done, end - 1, coverage * 100.0, planned_pts, poses.size());

    if (coverage < MIN_COVERAGE) {
      if (planned_pts == 0) {
        bool recovered = false;
        const size_t recovery_end = std::min(done + RECOVERY_LOOKAHEAD, total);

        for (size_t recovery_idx = done; recovery_idx < recovery_end; ++recovery_idx) {
          const auto recovery_pose = make_pose(
              toolpath.points[recovery_idx].x,
              toolpath.points[recovery_idx].y,
              toolpath.points[recovery_idx].z,
              tool_orientation(toolpath.points[recovery_idx]));

          moveit::planning_interface::MoveGroupInterface::Plan recovery_plan;
          double recovery_len = std::numeric_limits<double>::max();
          if (!plan_shortest_pose_target(
                  move_group, recovery_pose, recovery_plan, recovery_len)) {
            continue;
          }

          unwrap_trajectory(recovery_plan.trajectory_, move_group.getCurrentJointValues());
          if (move_group.execute(recovery_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            continue;
          }

          const size_t skipped_pts = recovery_idx - done;
          if (skipped_pts == 0) {
            RCLCPP_WARN(logger,
              "Cartesian planning failed at point %zu; recovered with joint-space planning.",
              done);
          } else {
            RCLCPP_WARN(logger,
              "Cartesian planning failed at point %zu; resumed at point %zu after skipping %zu unreachable waypoints.",
              done, recovery_idx, skipped_pts);
          }

          done = recovery_idx + 1;
          recovered = true;
          break;
        }

        if (recovered) {
          RCLCPP_INFO(logger, "Progress: %zu / %zu points (%.1f %%)",
                      done, total,
                      100.0 * static_cast<double>(done) / static_cast<double>(total));
          continue;
        }

        RCLCPP_ERROR(logger,
          "No waypoints could be planned at point %zu — possible collision "
          "or unreachable pose. Stopping.", done);
        break;
      }
      RCLCPP_WARN(logger,
        "Low coverage (%.1f %%) at batch [%zu–%zu] — executing the %zu "
        "planned waypoints and continuing.",
        coverage * 100.0, done, end - 1, planned_pts);
    }

    // Apply time parameterization (computeCartesianPath provides positions only)
    if (!retime_trajectory(trajectory, move_group.getRobotModel(),
                            *move_group.getCurrentState())) {
      RCLCPP_ERROR(logger, "Time parameterization failed for batch starting at %zu", done);
      break;
    }

    // Execute
    unwrap_trajectory(trajectory, move_group.getCurrentJointValues());
    auto rc = move_group.execute(trajectory);
    if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution failed at batch starting at point %zu", done);
      break;
    }

    printed_segments.push_back({done, done + planned_pts});
    publish_printed_segments(
        printed_part_publisher,
        move_group.getPlanningFrame(),
        toolpath.points,
        printed_segments);

    done = done + planned_pts;
    RCLCPP_INFO(logger, "Progress: %zu / %zu points (%.1f %%)",
                done, total, 100.0 * static_cast<double>(done) / static_cast<double>(total));
  }

  RCLCPP_INFO(logger, "Print complete — %zu / %zu points executed.", done, total);

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
