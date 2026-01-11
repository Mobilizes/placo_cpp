#pragma once

#include <placo/humanoid/footsteps_planner_repetitive.h>
#include <placo/humanoid/humanoid_parameters.h>
#include <placo/humanoid/humanoid_robot.h>
#include <placo/humanoid/walk_pattern_generator.h>
#include <placo/humanoid/walk_tasks.h>
#include <placo/kinematics/kinematics_solver.h>
#include <placo/tools/utils.h>
#include <termios.h>

#include <Eigen/Dense>
#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tachimawari_interfaces/msg/joint.hpp>
#include <tachimawari_interfaces/msg/set_joints.hpp>
#include <unordered_map>
#include <vector>

class PlacoCppNode : public rclcpp::Node
{
public:
  PlacoCppNode(std::string urdf_path, std::string config_path_in);
  ~PlacoCppNode();

private:
  // Internal helpers
  void update_loop();
  void handleWalkTeleop();
  void publish_joints();

  void setNonBlockingInput(bool enable);
  int readKey();

  void load_configuration();
  void save_configuration();
  void sync_configuration();

  // ROS2
  rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr joints_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // PLACO Objects
  std::unique_ptr<placo::humanoid::HumanoidRobot> robot;
  placo::humanoid::HumanoidParameters parameters;
  std::unique_ptr<placo::kinematics::KinematicsSolver> solver;
  std::unique_ptr<placo::humanoid::WalkTasks> tasks;
  std::unique_ptr<placo::kinematics::FrameTask> left_foot_task;
  std::unique_ptr<placo::kinematics::FrameTask> right_foot_task;
  std::unique_ptr<placo::kinematics::CoMTask> com_task;
  std::unique_ptr<placo::kinematics::OrientationTask> trunk_task;
  std::unique_ptr<placo::humanoid::FootstepsPlannerRepetitive> planner;
  std::unique_ptr<placo::humanoid::WalkPatternGenerator> pattern;

  placo::humanoid::WalkPatternGenerator::Trajectory trajectory;
  std::vector<placo::humanoid::FootstepsPlanner::Footstep> footsteps;
  std::vector<placo::humanoid::FootstepsPlanner::Support> supports;

  // Joint Map
  std::unordered_map<std::string, uint8_t> joints_name;

  // Teleop Variables
  double walk_dx_ = 0.0;
  double walk_dy_ = 0.0;
  double walk_dtheta_ = 0.0;

  // Parameters
  std::string config_path;
  double single_support_duration;
  double single_support_timesteps;
  double double_support_ratio;
  double startend_double_support_ratio;
  double planned_timesteps;
  double walk_com_height;
  double walk_foot_height;
  double walk_trunk_pitch;
  double walk_foot_rise_ratio;
  double foot_length;
  double foot_width;
  double feet_spacing;
  double zmp_margin;
  double foot_zmp_target_x;
  double foot_zmp_target_y;
  double walk_max_dtheta;
  double walk_max_dy;
  double walk_max_dx_forward;
  double walk_max_dx_backward;

  bool stop_walk;
  bool reset_pose;
};
