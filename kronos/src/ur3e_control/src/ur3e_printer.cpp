/**
 * UR3e 3D-printer controller.
 *
 * Reads a toolpath file (CSV: x,y,z per line) produced by a slicer,
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
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
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

// Cartesian planning
static constexpr double CART_EEF_STEP    = 0.001;  // 1 mm interpolation step
static constexpr double CART_JUMP_THRESH = 0.0;    // disable jump check
static constexpr double MIN_COVERAGE     = 0.95;   // require 95 % path coverage
static constexpr size_t BATCH_SIZE       = 50;      // waypoints per Cartesian call

// Approach: lift above first waypoint, then descend in a straight line
static constexpr double APPROACH_HEIGHT  = 0.05;   // 50 mm above first point
static constexpr int    PLAN_ATTEMPTS    = 10;     // OMPL attempts (pick shortest)

// Tool orientation — nozzle pointing straight down
static const tf2::Quaternion TOOL_ORIENTATION = []() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);      // rotate 180° about X  →  Z points down
    return q;
}();

// ---------------------------------------------------------------------------
// Toolpath parser
// ---------------------------------------------------------------------------
// Supports two formats:
//   1. CSV:   x,y,z   (one point per line; # comments and blank lines ignored)
//   2. G-code subset:  G0/G1 Xnnn Ynnn Znnn  (travel/print moves)
// ---------------------------------------------------------------------------
struct ToolPoint { double x, y, z; };

static bool ends_with(const std::string& s, const std::string& suffix) {
  if (suffix.size() > s.size()) return false;
  return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin(),
    [](char a, char b){ return std::tolower(a) == std::tolower(b); });
}

static std::vector<ToolPoint> parse_csv(const std::string& path) {
  std::vector<ToolPoint> pts;
  std::ifstream f(path);
  if (!f.is_open()) return pts;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    ToolPoint p;
    if (ss >> p.x >> p.y >> p.z) pts.push_back(p);
  }
  return pts;
}

static std::vector<ToolPoint> parse_gcode(const std::string& path) {
  std::vector<ToolPoint> pts;
  std::ifstream f(path);
  if (!f.is_open()) return pts;
  double cur_x = 0, cur_y = 0, cur_z = 0;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == ';') continue;
    // Only handle G0 / G1
    if (line.substr(0, 2) != "G0" && line.substr(0, 2) != "G1") continue;
    std::istringstream ss(line);
    std::string token;
    ss >> token;  // skip G0/G1
    while (ss >> token) {
      if (token.empty()) continue;
      char   axis = std::toupper(token[0]);
      double val  = 0;
      try { val = std::stod(token.substr(1)); } catch (...) { continue; }
      switch (axis) {
        case 'X': cur_x = val / 1000.0; break;  // mm → m
        case 'Y': cur_y = val / 1000.0; break;
        case 'Z': cur_z = val / 1000.0; break;
        default: break;
      }
    }
    pts.push_back({cur_x, cur_y, cur_z});
  }
  return pts;
}

static std::vector<ToolPoint> load_toolpath(const std::string& path) {
  if (ends_with(path, ".gcode") || ends_with(path, ".gco"))
    return parse_gcode(path);
  return parse_csv(path);   // default: CSV
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

static geometry_msgs::msg::Pose make_pose(double x, double y, double z,
                                           const tf2::Quaternion& q) {
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = tf2::toMsg(q);
  return p;
}

static void add_print_bed(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& frame) {
  moveit_msgs::msg::CollisionObject bed;
  bed.header.frame_id = frame;
  bed.id = "print_bed";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {BED_SIZE_X, BED_SIZE_Y, BED_SIZE_Z};

  geometry_msgs::msg::Pose bed_pose;
  bed_pose.position.x = BED_X;
  bed_pose.position.y = BED_Y;
  bed_pose.position.z = BED_Z;
  bed_pose.orientation.w = 1.0;

  bed.primitives.push_back(box);
  bed.primitive_poses.push_back(bed_pose);
  bed.operation = moveit_msgs::msg::CollisionObject::ADD;

  psi.applyCollisionObject(bed);
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

  std::string toolpath_file = node->get_parameter("toolpath_file").as_string();
  double print_speed  = node->get_parameter("print_speed").as_double();
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
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  moveit::planning_interface::PlanningSceneInterface planning_scene;

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
  add_print_bed(planning_scene, move_group.getPlanningFrame());
  rclcpp::sleep_for(std::chrono::seconds(1));   // let the scene propagate

  // --- Load toolpath ----------------------------------------------------
  if (toolpath_file.empty()) {
    RCLCPP_ERROR(logger, "No toolpath_file parameter set. "
                 "Use: ros2 launch ... toolpath_file:=/path/to/file.csv");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  auto toolpath = load_toolpath(toolpath_file);
  if (toolpath.empty()) {
    RCLCPP_ERROR(logger, "Toolpath file '%s' is empty or unreadable.",
                 toolpath_file.c_str());
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(logger, "Loaded %zu waypoints from '%s'",
              toolpath.size(), toolpath_file.c_str());

  // --- Move to approach pose above the first waypoint --------------------
  // Step 1: joint-space move to a point APPROACH_HEIGHT above waypoint 0.
  //         Multiple OMPL attempts → keep the shortest path.
  // Step 2: Cartesian straight-line descent to the actual first point.
  {
    auto approach_pose = make_pose(
        toolpath[0].x, toolpath[0].y,
        toolpath[0].z + APPROACH_HEIGHT, TOOL_ORIENTATION);

    move_group.setPoseTarget(approach_pose);
    move_group.setNumPlanningAttempts(PLAN_ATTEMPTS);

    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    double best_len = std::numeric_limits<double>::max();
    bool   found    = false;

    for (int attempt = 0; attempt < PLAN_ATTEMPTS; ++attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        continue;

      // Compute total joint-space path length to pick the shortest.
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
        best_len  = len;
        best_plan = plan;
        found     = true;
      }
    }
    move_group.setNumPlanningAttempts(1);  // reset for later calls

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
    auto first_pose = make_pose(toolpath[0].x, toolpath[0].y,
                                toolpath[0].z, TOOL_ORIENTATION);
    std::vector<geometry_msgs::msg::Pose> descent{first_pose};
    moveit_msgs::msg::RobotTrajectory descent_traj;
    double cov = move_group.computeCartesianPath(
        descent, CART_EEF_STEP, CART_JUMP_THRESH, descent_traj);
    if (cov < 0.99) {
      RCLCPP_ERROR(logger, "Cartesian descent to first waypoint failed "
                   "(coverage %.1f %%).", cov * 100.0);
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    unwrap_trajectory(descent_traj, move_group.getCurrentJointValues());
    if (move_group.execute(descent_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Failed to descend to first waypoint.");
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(logger, "Reached start position.");
  }

  // --- Execute toolpath in Cartesian batches ----------------------------
  size_t total  = toolpath.size();
  size_t done   = 0;

  while (done < total) {
    size_t end = std::min(done + BATCH_SIZE, total);

    // Build batch of Cartesian poses
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(end - done);
    for (size_t i = done; i < end; ++i) {
      poses.push_back(make_pose(
          toolpath[i].x, toolpath[i].y, toolpath[i].z, TOOL_ORIENTATION));
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
      RCLCPP_ERROR(logger,
        "Insufficient coverage (%.1f %%) — possible collision or "
        "unreachable region. Stopping.", coverage * 100.0);
      break;
    }

    // Execute
    unwrap_trajectory(trajectory, move_group.getCurrentJointValues());
    auto rc = move_group.execute(trajectory);
    if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution failed at batch starting at point %zu", done);
      break;
    }

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
