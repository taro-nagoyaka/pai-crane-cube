// Derived from CRANE+ pick_and_place.cpp, Copyright 2020 RT Corporation.
// Licensed under the Apache License, Version 2.0:
// http://www.apache.org/licenses/LICENSE-2.0
// Distributed on an AS IS BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
// Changes: check planning/execution results, publish reference trajectories,
// replace placing with upright lift, hold without releasing the cube,
// and constrain joints 2/3 to the negative elbow branch observed in training.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using Group = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("cube_lift_teacher", options);
  auto status = node->create_publisher<std_msgs::msg::String>(
    "/crane_cube_teacher/status", rclcpp::QoS(10).reliable().transient_local());
  auto reference = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/crane_cube_teacher/reference", rclcpp::QoS(20).reliable().transient_local());
  std::thread spin([&]() {rclcpp::spin(node);});
  auto announce = [&](const std::string & value) {
    std_msgs::msg::String msg;
    msg.data = value;
    status->publish(msg);
    RCLCPP_INFO(node->get_logger(), "%s", value.c_str());
  };

  int code = 0;
  try {
    // Batch-only options come from the batch runner, not from edited installed packages.
    auto setting = [](const char * key, double fallback, double limit) {
      const auto raw = std::getenv(key);
      if (!raw) {return fallback;}
      char * end = nullptr;
      const double value = std::strtod(raw, &end);
      if (end == raw || *end != '\0' || !std::isfinite(value) || std::abs(value) > limit) {
        throw std::runtime_error(std::string("Invalid teacher setting: ") + key);
      }
      return value;
    };
    const double cube_x = setting("CUBE_TEACHER_DX", 0.0, 0.004);
    const double cube_y = -0.1 + setting("CUBE_TEACHER_DY", 0.0, 0.004);
    const double radius = std::hypot(cube_x, cube_y);
    const double wrist_x = cube_x * (radius - 0.01) / radius;
    const double wrist_y = cube_y * (radius - 0.01) / radius;
    const double yaw = std::atan2(cube_y, cube_x);
    const double home_offset = setting("CUBE_TEACHER_HOME_OFFSET", 0.0, 0.06);
    const double lift_pitch = setting("CUBE_TEACHER_LIFT_PITCH", 0.0, 0.10);
    const double hold_seconds = setting("CUBE_TEACHER_HOLD_SECONDS", 2.0, 5.0);
    if (hold_seconds < 2.0) {throw std::runtime_error("Hold must be >= 2 simulated seconds");}
    Group arm(node, "arm");
    Group hand(node, "gripper");
    for (auto * group : {&arm, &hand}) {
      group->setMaxVelocityScalingFactor(0.35);
      group->setMaxAccelerationScalingFactor(0.35);
      group->setPlanningTime(10.0);
      group->setNumPlanningAttempts(5);
    }
    // Both collected episodes succeeded physically, but used different IK branches.
    // Keep the initial fixed-position imitation task on the negative branch.
    // These are planning/path constraints; no clipping or relabeling of old data.
    moveit_msgs::msg::Constraints branch;
    branch.name = "negative_elbow_branch";
    auto constrain = [&](const std::string & name, double lower, double upper) {
      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = name;
      constraint.position = (lower + upper) / 2.0;
      constraint.tolerance_below = (upper - lower) / 2.0;
      constraint.tolerance_above = (upper - lower) / 2.0;
      constraint.weight = 1.0;
      branch.joint_constraints.push_back(constraint);
    };
    // Limits are inside the user's URDF and include vertical=0 and home poses.
    constrain("crane_plus_joint2", -1.80, 0.02);
    constrain("crane_plus_joint3", -2.53, 0.02);
    arm.setPathConstraints(branch);
    RCLCPP_INFO(node->get_logger(), "Teacher uses constrained negative elbow branch");
    // This delay gives the already-running recorder time to receive status.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    announce("recording");

    auto execute = [&](Group & group, const std::string & label) {
      announce("stage:" + label);
      group.setStartStateToCurrentState();
      Group::Plan plan;
      if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Planning failed: " + label);
      }
      auto trajectory = plan.trajectory_.joint_trajectory;
      if (trajectory.points.empty()) {
        RCLCPP_INFO(node->get_logger(), "No motion required: %s", label.c_str());
        return;
      }
      // Same ROS simulation clock as images/states. Includes planning-to-controller latency.
      trajectory.header.stamp = node->now();
      reference->publish(trajectory);
      if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("Execution failed: " + label);
      }
    };
    auto named = [&](const std::string & name) {
      arm.clearPoseTargets();
      if (!arm.setNamedTarget(name)) {
        throw std::runtime_error("Unknown arm named pose: " + name);
      }
      execute(arm, name);
    };
    auto grip = [&](double angle, const std::string & label) {
      auto q = hand.getCurrentJointValues();
      if (q.size() != 1) {
        throw std::runtime_error("Expected one gripper joint");
      }
      q[0] = angle;
      hand.setJointValueTarget(q);
      execute(hand, label);
    };
    auto pose = [&](double z, double pitch, const std::string & label) {
      geometry_msgs::msg::Pose target;
      target.position.x = wrist_x;
      target.position.y = wrist_y;
      target.position.z = z;
      tf2::Quaternion q;
      q.setRPY(0.0, pitch, yaw);
      target.orientation = tf2::toMsg(q);
      arm.clearPoseTargets();
      if (!arm.setPoseTarget(target)) {
        throw std::runtime_error("Pose target invalid: " + label);
      }
      execute(arm, label);
    };

    // Approach/grasp waypoints from the Humble CRANE+ official sample.
    // Positions use the robot planning frame, NOT Gazebo's elevated table world frame.
    named("vertical");
    grip(0.0, "gripper_default");
    named("home");
    if (home_offset != 0.0) {
      // Teach moving out of nearby home poses rather than holding there indefinitely.
      arm.setJointValueTarget(std::map<std::string, double>{
        {"crane_plus_joint1", home_offset},
        {"crane_plus_joint2", -1.16 + home_offset * 0.5},
        {"crane_plus_joint3", -2.01 + home_offset},
        {"crane_plus_joint4", -0.73 - home_offset * 0.5}});
      execute(arm, "near_home");
    }
    grip(-M_PI / 6, "open");
    pose(0.17, M_PI / 2, "approach");
    pose(0.17, M_PI, "face_down");
    pose(0.14, M_PI, "descend");
    grip(M_PI / 18, "grasp");
    if (lift_pitch != 0.0) {
      // A closed-gripper lift through a slightly tilted intermediate pose.
      // This can fail physically; the batch recorder must reject dropped cubes.
      pose(0.155, M_PI + lift_pitch, "lift_recovery");
    }
    pose(0.17, M_PI, "lift_clear");
    // No place / release. Carry via the existing home pose, then raise upright.
    named("home");
    named("vertical");
    announce("stage:hold_two_sim_seconds");
    const auto start = node->now();
    const auto wall_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (node->now() - start).seconds() < hold_seconds) {
      if (std::chrono::steady_clock::now() - wall_start > std::chrono::seconds(60)) {
        throw std::runtime_error("Simulation clock stopped during hold");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted");
    }
    announce("done");
    // Allow final status to reach recorder before node destruction.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } catch (const std::exception & e) {
    code = 1;
    announce(std::string("failed:") + e.what());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  rclcpp::shutdown();
  if (spin.joinable()) {spin.join();}
  return code;
}
