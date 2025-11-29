#include "placo_cpp_node.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

#include "keisan/keisan.hpp"
#include "jitsuyo/jitsuyo.hpp"

using namespace std::chrono_literals;

PlacoCppNode::PlacoCppNode() : Node("placo_cpp_node")
{
    joints_pub_ = this->create_publisher<tachimawari_interfaces::msg::SetJoints>(
        "/joint/set_joints", 10);

    RCLCPP_INFO(this->get_logger(), "Loading robot model...");

    const double DT = 0.005;
    const std::string model_filename = "absolute/path/to/robot.urdf";
    config_path = "/absolute/path/to/config/directory/";

    robot = std::make_unique<placo::humanoid::HumanoidRobot>(model_filename);
    parameters = placo::humanoid::HumanoidParameters();

    load_configuration();

    // Set walking parameters
    parameters.single_support_duration = single_support_duration;
    parameters.single_support_timesteps = single_support_timesteps;
    parameters.double_support_ratio = double_support_ratio;
    parameters.startend_double_support_ratio = startend_double_support_ratio;
    parameters.planned_timesteps = planned_timesteps;

    parameters.walk_com_height = walk_com_height;
    parameters.walk_foot_height = walk_foot_height;
    parameters.walk_trunk_pitch = walk_trunk_pitch;
    parameters.walk_foot_rise_ratio = walk_foot_rise_ratio;

    parameters.foot_length = foot_length;
    parameters.foot_width = foot_width;
    parameters.feet_spacing = feet_spacing;
    parameters.zmp_margin = zmp_margin;
    parameters.foot_zmp_target_x = foot_zmp_target_x;
    parameters.foot_zmp_target_y = foot_zmp_target_y;

    parameters.walk_max_dtheta = walk_max_dtheta;
    parameters.walk_max_dy = walk_max_dy;
    parameters.walk_max_dx_forward = walk_max_dx_forward;
    parameters.walk_max_dx_backward = walk_max_dx_backward;

    // Solver
    solver = std::make_unique<placo::kinematics::KinematicsSolver>(*robot);
    solver->dt = DT;
    solver->enable_velocity_limits(true);

    tasks = std::make_unique<placo::humanoid::WalkTasks>();
    tasks->initialize_tasks(solver.get(), robot.get());

    auto joints_task = solver->add_joints_task();
    joints_task.configure("joints", "soft", 1.0);

    // Initial pose
    Eigen::Affine3d eye = Eigen::Affine3d::Identity();
    tasks->reach_initial_pose(eye, parameters.feet_spacing,
                              parameters.walk_com_height, parameters.walk_trunk_pitch);

    // Walk planner
    planner = std::make_unique<placo::humanoid::FootstepsPlannerRepetitive>(parameters);
    pattern = std::make_unique<placo::humanoid::WalkPatternGenerator>(*robot, parameters);

    double walk_dx = 0.0, walk_dy = 0.0, walk_dtheta = 0.0;
    int nb_steps = 10;
    planner->configure(walk_dx, walk_dy, walk_dtheta, nb_steps);

    Eigen::Affine3d T_world_left = placo::tools::flatten_on_floor(robot->get_T_world_left());
    Eigen::Affine3d T_world_right = placo::tools::flatten_on_floor(robot->get_T_world_right());

    footsteps = planner->plan(
        placo::humanoid::HumanoidRobot::Side::Left,
        T_world_left, T_world_right);

    supports = placo::humanoid::FootstepsPlannerRepetitive::make_supports(
        footsteps, 0.0, true, parameters.has_double_support(), true);

    trajectory = pattern->plan(supports, robot->com_world(), 0.0);

    // joint ID mapping
    joints_name = {
        {"head_yaw", 19}, {"head_pitch", 20},
        {"left_shoulder_pitch", 2}, {"left_shoulder_roll", 4}, {"left_elbow", 6},
        {"right_shoulder_pitch", 1}, {"right_shoulder_roll", 3}, {"right_elbow", 5},
        {"left_hip_yaw", 8}, {"left_hip_roll", 10}, {"left_hip_pitch", 12},
        {"left_knee", 14}, {"left_ankle_pitch", 16}, {"left_ankle_roll", 18},
        {"right_hip_yaw", 7}, {"right_hip_roll", 9}, {"right_hip_pitch", 11},
        {"right_knee", 13}, {"right_ankle_pitch", 15}, {"right_ankle_roll", 17}
    };

    // non-blocking keyboard
    setNonBlockingInput(true);

    timer_ = this->create_wall_timer(5ms, std::bind(&PlacoCppNode::update_loop, this));

    RCLCPP_INFO(this->get_logger(), "PlacoCppNode initialized.");
}

PlacoCppNode::~PlacoCppNode()
{
    setNonBlockingInput(false);
}

void PlacoCppNode::load_configuration()
{
  nlohmann::json config;
  if (!jitsuyo::load_config(config_path, "placo.json", config)) {
    throw std::runtime_error("Failed to load config file `" + config_path + "placo.json`");
  }

  bool valid_config = true;

  nlohmann::json walking_section;
  if (jitsuyo::assign_val(config, "walking", walking_section)) {
    bool valid_section = jitsuyo::assign_val(walking_section, "single_support_duration", single_support_duration);
    valid_section &= jitsuyo::assign_val(walking_section, "single_support_timesteps", single_support_timesteps);
    valid_section &= jitsuyo::assign_val(walking_section, "double_support_ratio", double_support_ratio);
    valid_section &= jitsuyo::assign_val(walking_section, "startend_double_support_ratio", startend_double_support_ratio);
    valid_section &= jitsuyo::assign_val(walking_section, "planned_timesteps", planned_timesteps);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_com_height", walk_com_height);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_foot_height", walk_foot_height);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_trunk_pitch", walk_trunk_pitch);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_foot_rise_ratio", walk_foot_rise_ratio);
    valid_section &= jitsuyo::assign_val(walking_section, "foot_length", foot_length);
    valid_section &= jitsuyo::assign_val(walking_section, "foot_width", foot_width);
    valid_section &= jitsuyo::assign_val(walking_section, "feet_spacing", feet_spacing);
    valid_section &= jitsuyo::assign_val(walking_section, "zmp_margin", zmp_margin);
    valid_section &= jitsuyo::assign_val(walking_section, "foot_zmp_target_x", foot_zmp_target_x);
    valid_section &= jitsuyo::assign_val(walking_section, "foot_zmp_target_y", foot_zmp_target_y);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_max_dtheta", walk_max_dtheta);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_max_dy", walk_max_dy);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_max_dx_forward", walk_max_dx_forward);
    valid_section &= jitsuyo::assign_val(walking_section, "walk_max_dx_backward", walk_max_dx_backward);

    if (!valid_section) {
      jitsuyo::print("Error found at section `walking`");
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `" + config_path + "placo.json`");
  }
}

void PlacoCppNode::setNonBlockingInput(bool enable)
{
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

int PlacoCppNode::readKey()
{
    unsigned char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1)
        return ch;
    return -1;
}

void PlacoCppNode::update_loop()
{
    const double DT = 0.005;
    static double t = 0.0;
    static double last_replan = -1e9;
    handleWalkTeleop();

    tasks->update_tasks(trajectory, t);
    robot->update_kinematics();
    solver->solve(true);

    if (!trajectory.support_is_both(t)) {
        robot->update_support_side(trajectory.support_side(t));
        robot->ensure_on_floor();
    }

    const double REPLAN_DT = 0.1;
    int nb_steps = 10;
    if ((t - last_replan > REPLAN_DT) && pattern->can_replan_supports(trajectory, t)) {
        planner->configure(walk_dx_, walk_dy_, walk_dtheta_, nb_steps);
        supports = pattern->replan_supports(*planner, trajectory, t, last_replan);
        trajectory = pattern->replan(supports, trajectory, t);
        last_replan = t;
    }

    publish_joints();
    t += DT;
}

void PlacoCppNode::handleWalkTeleop()
{
    int key = readKey();
    if (key == -1) return;

    const double speed_step = 0.025;
    const double turn_step = 0.1;

    switch (key) {
        case 'w': walk_dx_ = std::min(walk_dx_ + speed_step, parameters.walk_max_dx_forward); break;
        case 's': walk_dx_ = std::max(walk_dx_ - speed_step, -parameters.walk_max_dx_backward); break;
        case 'j': walk_dy_ = std::min(walk_dy_ + speed_step, parameters.walk_max_dy); break;
        case 'l': walk_dy_ = std::max(walk_dy_ - speed_step, -parameters.walk_max_dy); break;
        case 'a': walk_dtheta_ = std::min(walk_dtheta_ + turn_step, parameters.walk_max_dtheta); break;
        case 'd': walk_dtheta_ = std::max(walk_dtheta_ - turn_step, -parameters.walk_max_dtheta); break;
        case ' ': walk_dx_ = walk_dy_ = walk_dtheta_ = 0.0; break;
        default: break;
    }

    std::cout << "[ctrl] dx=" << walk_dx_
              << ", dy=" << walk_dy_
              << ", dθ=" << walk_dtheta_ << std::endl;
}

void PlacoCppNode::publish_joints()
{
    auto msg = tachimawari_interfaces::msg::SetJoints();
    msg.control_type = 4;

    // Arm pose
    const double elbow = -50.0 * M_PI / 180.0;
    const double shoulder_roll = -80.0 * M_PI / 180.0;
    const double shoulder_pitch = 20.0 * M_PI / 180.0;

    robot->set_joint("left_shoulder_roll", shoulder_roll);
    robot->set_joint("left_shoulder_pitch", shoulder_pitch);
    robot->set_joint("left_elbow", elbow);

    robot->set_joint("right_shoulder_roll", -shoulder_roll);
    robot->set_joint("right_shoulder_pitch", shoulder_pitch);
    robot->set_joint("right_elbow", elbow);

    robot->set_joint("head_pitch", 0.0);
    robot->set_joint("head_yaw", 0.0);

    for (const auto &pair : joints_name) {
        const std::string &name = pair.first;
        uint8_t id = pair.second;

        tachimawari_interfaces::msg::Joint j;
        j.id = id;
        j.position = robot->get_joint(name);
        msg.joints.push_back(j);
    }

    joints_pub_->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlacoCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
