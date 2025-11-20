#include <rclcpp/rclcpp.hpp>
#include <tachimawari_interfaces/msg/set_joints.hpp>
#include <tachimawari_interfaces/msg/joint.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <mutex>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <placo/humanoid/humanoid_robot.h>
#include <placo/humanoid/humanoid_parameters.h>
#include <placo/humanoid/walk_pattern_generator.h>
#include <placo/humanoid/walk_tasks.h>
#include <placo/humanoid/footsteps_planner_repetitive.h>
#include <placo/kinematics/kinematics_solver.h>
#include <placo/tools/utils.h>

using std::placeholders::_1;

class PlacoCppNode : public rclcpp::Node
{
public:
    PlacoCppNode() : Node("placo_cpp_node")
    {
        using namespace std::chrono_literals;

        // === Publisher ===
        joints_pub_ = this->create_publisher<tachimawari_interfaces::msg::SetJoints>(
            "/joint/set_joints", 10);

        RCLCPP_INFO(this->get_logger(), "Loading robot model...");

        // === PLACO INIT ===
        const double DT = 0.005;
        const std::string model_filename = "/home/mbsaloka/placo_ws/hiro-urdf/robot.urdf";

        robot = std::make_unique<placo::humanoid::HumanoidRobot>(model_filename);
        parameters = placo::humanoid::HumanoidParameters();

        parameters.single_support_duration = 0.33;
        parameters.single_support_timesteps = 10;
        parameters.double_support_ratio = 0.3;
        parameters.startend_double_support_ratio = 1.5;
        parameters.planned_timesteps = 48;

        parameters.walk_com_height = 0.26;
        parameters.walk_foot_height = 0.04;
        parameters.walk_trunk_pitch = 0.0;
        parameters.walk_foot_rise_ratio = 0.2;

        parameters.foot_length = 0.142;
        parameters.foot_width = 0.083;
        parameters.feet_spacing = 0.13;
        parameters.zmp_margin = 0.01;
        parameters.foot_zmp_target_x = 0.0;
        parameters.foot_zmp_target_y = 0.0;

        parameters.walk_max_dtheta = 1.5;
        parameters.walk_max_dy = 0.1;
        parameters.walk_max_dx_forward = 0.175;
        parameters.walk_max_dx_backward = 0.175;

        solver = std::make_unique<placo::kinematics::KinematicsSolver>(*robot);
        solver->dt = DT;
        solver->enable_velocity_limits(true);

        tasks = std::make_unique<placo::humanoid::WalkTasks>();
        tasks->initialize_tasks(solver.get(), robot.get());

        auto joints_task = solver->add_joints_task();
        joints_task.configure("joints", "soft", 1.0);

        // === Initial pose ===
        Eigen::Affine3d eye = Eigen::Affine3d::Identity();
        tasks->reach_initial_pose(eye, parameters.feet_spacing,
                                  parameters.walk_com_height, parameters.walk_trunk_pitch);

        // === Walk planner ===
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

        timer_ = this->create_wall_timer(
            5ms, std::bind(&PlacoCppNode::update_loop, this));

        // non-blocking keyboard
        setNonBlockingInput(true);

        timer_ = this->create_wall_timer(
            5ms, std::bind(&PlacoCppNode::update_loop, this));

        RCLCPP_INFO(this->get_logger(), "PlacoCppNode initialized.");
    }

    ~PlacoCppNode() { setNonBlockingInput(false); }

private:

    void setNonBlockingInput(bool enable)
    {
        static struct termios oldt, newt;
        if (enable)
        {
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        }
        else
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }
    }

    int readKey()
    {
        unsigned char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1)
            return ch;
        return -1;
    }

    void update_loop()
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


    void handleWalkTeleop()
    {
        int key = readKey();
        if (key == -1) return;

        const double speed_step = 0.025;
        const double turn_step = 0.1;

        switch (key)
        {
            case 'w': walk_dx_ = std::min(walk_dx_ + speed_step, parameters.walk_max_dx_forward); break;
            case 's': walk_dx_ = std::max(walk_dx_ - speed_step, -parameters.walk_max_dx_backward); break;
            case 'j': walk_dy_ = std::min(walk_dy_ + speed_step, parameters.walk_max_dy); break;
            case 'l': walk_dy_ = std::max(walk_dy_ - speed_step, -parameters.walk_max_dy); break;
            case 'a': walk_dtheta_ = std::min(walk_dtheta_ + turn_step, parameters.walk_max_dtheta); break;
            case 'd': walk_dtheta_ = std::max(walk_dtheta_ - turn_step, -parameters.walk_max_dtheta); break;
            case ' ': walk_dx_ = walk_dy_ = walk_dtheta_ = 0.0; break;
            default: break;
        }

        // Debug output
        std::cout << "[ctrl] dx=" << walk_dx_
                  << ", dy=" << walk_dy_
                  << ", dθ=" << walk_dtheta_ << std::endl;
    }

    void publish_joints()
    {
        auto msg = tachimawari_interfaces::msg::SetJoints();
        msg.control_type = 4;

        // --- Set default joint positions ---
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

        // --- Publish all joints ---
        std::map<std::string, double> joints;
        for (const auto &pair : joints_name)
        {
            const std::string &joint_name = pair.first;
            uint8_t joint_id = pair.second;
            double joint_pos = robot->get_joint(joint_name);
            joints[joint_name] = joint_pos;

            // Debug (optional)
            // RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_name.c_str(), joint_pos);

            tachimawari_interfaces::msg::Joint j;
            j.id = joint_id;
            j.position = joint_pos;
            msg.joints.push_back(j);
        }

        joints_pub_->publish(msg);
    }


    // ROS2
    rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr joints_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PLACO objects
    std::unique_ptr<placo::humanoid::HumanoidRobot> robot;
    placo::humanoid::HumanoidParameters parameters;
    std::unique_ptr<placo::kinematics::KinematicsSolver> solver;
    std::unique_ptr<placo::humanoid::WalkTasks> tasks;
    std::unique_ptr<placo::humanoid::FootstepsPlannerRepetitive> planner;
    std::unique_ptr<placo::humanoid::WalkPatternGenerator> pattern;

    placo::humanoid::WalkPatternGenerator::Trajectory trajectory;
    std::vector<placo::humanoid::FootstepsPlanner::Footstep> footsteps;
    std::vector<placo::humanoid::FootstepsPlanner::Support> supports;
    std::unordered_map<std::string, uint8_t> joints_name = {
        // head motors
        {"head_yaw", 19},
        {"head_pitch", 20},

        // left arm motors
        {"left_shoulder_pitch", 2},
        {"left_shoulder_roll", 4},
        // {"left_shoulder_yaw", 24},
        {"left_elbow", 6},
        // {"left_gripper", 22},

        // right arm motors
        {"right_shoulder_pitch", 1},
        {"right_shoulder_roll", 3},
        // {"right_shoulder_yaw", 23},
        {"right_elbow", 5},
        // {"right_gripper", 21},

        // left leg motors
        {"left_hip_yaw", 8},
        {"left_hip_roll", 10},
        {"left_hip_pitch", 12},
        {"left_knee", 14},
        {"left_ankle_pitch", 16},
        {"left_ankle_roll", 18},

        // right leg motors
        {"right_hip_yaw", 7},
        {"right_hip_roll", 9},
        {"right_hip_pitch", 11},
        {"right_knee", 13},
        {"right_ankle_pitch", 15},
        {"right_ankle_roll", 17}
    };


    // walking teleop
    double walk_dx_ = 0.0;
    double walk_dy_ = 0.0;
    double walk_dtheta_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlacoCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
