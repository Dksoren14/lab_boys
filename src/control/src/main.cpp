
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ctime>
#include <termios.h>
#include <fcntl.h>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

// Include message types
#include "interfaces/msg/base_state.hpp"
#include "interfaces/action/base_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

// Include own headers
#include "state_manager.h"
#include "controller.h"
#include "transformation.h"

using namespace std;


class LabBaseNode : public rclcpp::Node
{
public:
    using BaseCommandAction = interfaces::action::BaseCommand;
    using GoalHandleBaseCommand = rclcpp_action::ServerGoalHandle<BaseCommandAction>;
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    LabBaseNode() 
    : Node("lab_base_node"),
    state_manager(),
    controller(state_manager)
    {
        std::cout << "LabBaseNode initialized" << std::endl;
        
        this->declare_parameter<double>("turning_controller.kp", 1.0);
        this->declare_parameter<double>("turning_controller.kd", 0.1);
        this->declare_parameter<double>("angular_controller.kp", 1.0);
        this->declare_parameter<double>("angular_controller.kd", 0.1);
        this->declare_parameter<double>("linear_controller.kp", 1.0);
        this->declare_parameter<double>("linear_controller.kd", 0.1);
        this->declare_parameter<double>("linear_precision_controller.kp", 1.0);
        this->declare_parameter<double>("linear_precision_controller.kd", 0.5);
        this->declare_parameter<double>("target_distance.angular", 0.01);
        this->declare_parameter<double>("target_distance.waypoint", 0.01);
        this->declare_parameter<double>("goal_distance.threshold", 0.1);

        pid_turning_gains.kp = this->get_parameter("turning_controller.kp").as_double();
        pid_turning_gains.kd = this->get_parameter("turning_controller.kd").as_double();
        pid_angular_gains.kp = this->get_parameter("angular_controller.kp").as_double();
        pid_angular_gains.kd = this->get_parameter("angular_controller.kd").as_double();
        pid_linear_gains.kp = this->get_parameter("linear_controller.kp").as_double();
        pid_linear_gains.kd = this->get_parameter("linear_controller.kd").as_double();
        pid_linear_precision_gains.kp = this->get_parameter("linear_precision_controller.kp").as_double();
        pid_linear_precision_gains.kd = this->get_parameter("linear_precision_controller.kd").as_double();
        accepted_distance.angular = this->get_parameter("target_distance.angular").as_double();
        accepted_distance.waypoint = this->get_parameter("target_distance.waypoint").as_double();
        goal_distance.threshold = this->get_parameter("goal_distance.threshold").as_double();

        std::cout << "Controller gains: kp=" << pid_turning_gains.kp << ", kd=" << pid_turning_gains.kd << std::endl;
        std::cout << "Controller gains: kp=" << pid_angular_gains.kp << ", kd=" << pid_angular_gains.kd << std::endl;
        std::cout << "Controller gains: kp=" << pid_linear_gains.kp << ", kd=" << pid_linear_gains.kd << std::endl;
        std::cout << "Controller gains: kp=" << pid_linear_precision_gains.kp << ", kd=" << pid_linear_precision_gains.kd << std::endl;
        std::cout << "Target distances: angular=" << accepted_distance.angular << ", waypoint=" << accepted_distance.waypoint << std::endl;
        std::cout << "Goal distance: threshold=" << goal_distance.threshold << std::endl;
        std::cout << R"(

                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣈⣿⣿⣿⣿⣿⣇⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣤⣤⡀⠀⣠⣼⣿⣿⣿⠿⠿⢯⡿⠿⢤⡀⠀⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⡝⠀⠀⠀⠈⠸⠗⠽⣼⣿⣿⣿⣿⢆⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣴⣷⣧⣤⡀⢰⣷⣻⣿⣿⣿⣿⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⢹⣿⣿⣿⣿⣿⡝⣴⣿⣿⣿⣯⣍⠀⠈⡌⡟⠛⠛⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⠣⣺⣗⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣟⣝⢻⡿⠟⣻⡟⠋⠉⢋⡟⣷⠎⣄⢁⠊⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⡇⠀⡀⣿⡧⠀⡴⡾⠰⠁⢙⣦⣭⣼⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⣾⣄⣿⣿⣾⣆⣤⣖⣠⣾⡗⣾⡿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⢰⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⢀⡠⠤⣠⣶⣝⣿⣿⣿⣿⡛⠑⢩⣿⡿⠓⠛⠉⣁⢎⣮⣰⣿⣏⢶⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⣠⣾⣷⣿⣶⣿⣿⣿⣿⣿⣿⣿⣿⡓⣿⣿⣿⠀⠀⣀⣷⢿⣿⣿⣿⠩⢙⣯⣗⣎⣭⣲⣄⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⣠⣾⣿⣿⣿⣯⣽⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⡶⢻⢿⣿⣿⣿⣿⣩⠀⠄⠰⠍⢢⠀⠀⠀⠀
        ⠀⠀⢀⣼⣿⣿⣿⣿⣿⣿⣇⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣭⠁⠈⠙⢻⣋⣹⠐⠀⣿⣿⣿⣿⣷⣖⠁⡀⠀⠓⢣⠀⠀⠀
        ⠀⣠⠾⠙⠻⣿⣿⣿⣿⣿⣿⣿⡿⢻⣿⣿⣿⣿⣿⣿⡿⠿⠁⠀⠀⠀⠹⢁⠑⠀⡘⡿⣿⣿⣿⣯⢂⣄⡀⠘⣳⡆⠀⠀
        ⣼⣷⣏⣠⣌⣨⣿⣿⣿⣿⡟⠋⢠⣿⣿⣿⣿⣿⣿⣿⣷⣮⠁⠀⠀⠀⠀⢣⣈⠁⢃⡹⣿⣿⣿⣿⣖⠎⠁⠀⠁⡸⡀⠀
        ⣿⣿⡿⣿⣯⡿⢵⣿⣿⣿⡶⢆⣾⣿⣿⣿⣿⣿⣿⠛⠛⠋⠀⠀⠀⠀⠀⠀⣗⠑⠈⠁⡹⣿⠻⣿⣿⣷⣵⣦⣤⠊⠁⠀
        ⠹⣿⣯⣽⣿⣾⣷⡟⠙⠻⢯⠋⣿⣿⣿⣿⣿⣿⣾⣤⡀⠀⠀⠀⠀⠀⠀⠀⢸⠙⠨⢔⠅⣿⡀⠈⣻⣿⣿⡇⠑⠫⠑⡄
        ⠀⠈⠻⢿⣿⣿⡟⠻⢦⣤⠃⢸⣿⣿⣿⣿⣿⣿⣿⡋⠊⠀⠀⠀⠀⠀⠀⠀⠈⣞⣣⢴⢺⣿⡇⢠⣿⣿⣿⠂⠂⠀⠃⢡
        ⠀⠀⠀⢸⣿⡿⢿⣤⣤⡜⠀⠘⣿⣿⣿⣿⣿⣿⡿⡋⡷⡂⢀⠀⠀⠀⡀⠀⠀⣗⣿⣓⣾⣿⠇⢸⣿⣿⣿⣧⠠⠀⢲⣸
        ⠀⠀⠀⠈⠻⢿⣦⣖⡺⠁⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣵⠐⡈⠀⠀⠀⠀⠀⠀⣷⣝⣄⣿⣿⠀⢸⣿⣿⣷⠊⠀⠀⣢⡇
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⡟⡻⢹⣿⠇⠀⠘⣿⣿⣷⣷⣥⣄⡿⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣽⣿⣿⣿⣿⣫⡟⠊⠁⠀⠀⠀⠀⠀⠀⢀⢗⣽⣾⣯⡄⠀⢀⣾⠿⡟⢿⢟⣋⠁⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣷⣧⣤⣤⣄⣠⡤⣴⣿⣿⣿⣿⣿⣷⣿⣟⢁⢸⠁⠤⣣⡟⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣿⣿⣿⣿⣿⣿⣿⣿⣤⢦⠜⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡅⠈⠀⠨⢪⢁⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣵⣿⣿⣿⣿⣿⣿⣿⣿⠿⢿⣷⡟⡠⡷⠁⢂⠧⢹⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⡻⢿⣿⣿⣿⣿⠿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣿⣿⣇⣽⡇⣠⣾⢹⡎⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⠥⠓⠊⠝⣻⣿⠀⠀⣿⣿⣿⣿⡿⡿⡿⠟⢑⣾⠀⠈⠛⠓⠚⠛⠚⠁⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⡊⢀⠀⠀⠐⣿⠀⠀⣿⣿⣿⡿⠉⢠⢁⡀⠼⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⡿⢒⡐⣄⣴⡿⠀⠀⢹⣿⣿⣿⢀⠀⠀⢀⣭⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡿⢾⠿⢿⡾⠁⠀⠀⠈⢿⣿⣿⣿⠷⠿⡾⣾⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣽⣿⣿⣿⣇⣠⣴⡎⠀⠀⠀⠀⢸⣿⣿⣿⣾⡅⣌⣿⡅⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⢏⠉⠉⢹⢷⠀⠀⠀⠀⢸⣿⣿⣿⡏⠁⠀⠪⣱⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡿⠿⠧⠀⠀⣺⠀⠀⠀⠀⢸⣿⣿⣿⢄⠀⠄⠌⡉⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡃⠀⡀⠀⢠⣿⠀⠀⠀⠀⢸⣿⣿⣿⡄⠗⠀⢀⣸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣿⣿⣷⢞⡔⠀⠺⡇⠀⠀⠀⠀⢸⣿⣿⣿⣧⡂⢀⠴⣶⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣿⠇⠀⢠⣃⠇⠀⠀⠀⠀⠀⣿⣿⣿⡟⣏⢀⣹⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠤⣶⣿⣿⣿⣿⣗⠒⢿⣷⡄⠀⠀⠀⣠⣾⣿⣿⣷⠚⢳⣾⣦⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⣠⣤⣶⣾⣥⡾⣿⠟⣫⣾⣿⣿⣄⣤⣿⣿⠀⠀⡼⣾⣿⣩⣿⣿⣷⣹⣿⡿⢮⣇⠀⠀⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣷⣾⣯⣸⣿⣿⡿⢉⣀⢹⣿⣿⣧⣾⣿⣯⣽⣿⣿⣿⠇⠉⣿⣧⣦⣱⡵⣆⠀⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠸⢿⣿⣿⣿⣿⣿⣿⡺⢼⣿⣿⣗⡊⣘⣹⡛⢋⣿⣿⣟⠪⢻⣿⣿⣿⠂⠧⣽⣿⣧⠁⠀⣟⡄⠀⠀⠀⠀⠀
        ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠀⠉⠉⠛⠓⠒⠒⠚⠁⠈⠛⠻⠧⠤⠛⠛⠿⠯⠿⠬⠞⠛⠻⠭⠝⠓⠂⠀
)" << std::endl;
        controller.setGains(pid_turning_gains, pid_linear_gains, pid_angular_gains, pid_linear_precision_gains);
        aruco_last_seen_timer = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        // --- Quality of Service settings for subscriptions ---
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // --- Subscriptions ---
        base_global_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                global_callback(msg);
            }
        ); 

        aruco_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "/aruco_marker_pose",
            qos,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg){
                aruco_callback(msg);
            }
        );


        // --- Publishers ---
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
         "/cmd_vel", 10);

        base_state_pub = this->create_publisher<interfaces::msg::BaseState>(
            "lab_boys/out/base_state", 10);

        // -- Action server ---
        base_command_server = rclcpp_action::create_server<BaseCommandAction>(
            this, "base_command",
            [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BaseCommandAction::Goal> goal)
            {return handle_goal(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandleBaseCommand> goal_handle)
            {return handle_cancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandleBaseCommand> goal_handle)
            {handle_accepted(goal_handle);
            }

        );

        // -- Action client ---
        path_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this,
            "/compute_path_to_pose");

        // --- Timers ---
        base_state_timer = create_wall_timer(20ms, [this](){ publish_base_state(); });
        path_timer = create_wall_timer(1s, [this](){ replan(); });
    }


private:
    StateManager state_manager;
    Controller controller;
    Transformation transformation;
    PIDControllerGains pid_linear_gains;
    PIDControllerGains pid_angular_gains;
    PIDControllerGains pid_linear_precision_gains;
    PIDControllerGains pid_turning_gains;
    TargetDistance accepted_distance;
    GoalDistance goal_distance;
    rclcpp::TimerBase::SharedPtr control_timer;
    PositionError previous_position_error;
    PositionError previous_angle_error;
    PositionError previous_velocity_error;
    Stamped3DVector previous_position;
    std::shared_ptr<const BaseCommandAction::Goal> current_goal;
    std::shared_ptr<BaseCommandAction::Result> current_result;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_global_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr aruco_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub; 
    rclcpp::Publisher<interfaces::msg::BaseState>::SharedPtr base_state_pub;
    rclcpp::Time last_time;
    rclcpp::TimerBase::SharedPtr base_state_timer;
    rclcpp::TimerBase::SharedPtr path_timer;
    rclcpp::Time aruco_last_seen_timer;
    bool reached_target_angle = false;
    int current_waypoint_idx_ = 0;

    rclcpp_action::Server<BaseCommandAction>::SharedPtr base_command_server;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client;
    nav2_msgs::action::ComputePathToPose::Goal goal_msg;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    Stamped3DVector goal_position;

    bool key_up = false, key_down = false, key_left = false, key_right = false, key_side_left = false, key_side_right = false;
    std::thread keyboard_thread_;
    bool keyboard_running_ = false;


    int control_mode = 0; // 0: idle, 1: goto, 2: stop

    void replan()
    {   
        
       
       if (!path_client->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Planner not available");
            return;
        }
        auto q = state_manager.getGlobalBaseOrientation();
        ComputePathToPose::Goal goal_msg;
        goal_msg.start.header.frame_id = "map";
        goal_msg.goal.header.frame_id  = "map";
        goal_msg.start.header.stamp = state_manager.getGlobalBasePosition().timestamp;
        goal_msg.goal.header.stamp = state_manager.getGoalPosition().timestamp;
        goal_msg.start.pose.position.x = state_manager.getGlobalBasePosition().x();
        goal_msg.start.pose.position.y = state_manager.getGlobalBasePosition().y();
        goal_msg.start.pose.position.z = state_manager.getGlobalBasePosition().z();
        goal_msg.start.pose.orientation.x = q.x();
        goal_msg.start.pose.orientation.y = q.y();
        goal_msg.start.pose.orientation.z = q.z();
        goal_msg.start.pose.orientation.w = q.w();
        

        goal_msg.goal.pose.position.x = state_manager.getGoalPosition().x();
        goal_msg.goal.pose.position.y = state_manager.getGoalPosition().y();
        goal_msg.goal.pose.position.z = state_manager.getGoalPosition().z();
        goal_msg.goal.pose.orientation.x = 0.0;
        goal_msg.goal.pose.orientation.y = 0.0;
        goal_msg.goal.pose.orientation.z = 0.0;
        goal_msg.goal.pose.orientation.w = 1.0;

        goal_msg.use_start = true;

        auto options =
            rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

        options.result_callback =
            std::bind(&LabBaseNode::path_callback, this, std::placeholders::_1);

        path_client->async_send_goal(goal_msg, options); 
        }



    //  --- Callbacks--
    void path_callback(
        const rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult & result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(), "Path planning failed");
            return;
        }

        auto path = result.result->path;

        //Guard for empty path
        if (path.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received path with %ld poses", path.poses.size());

        Stamped3DVector temp_global = state_manager.getGlobalBasePosition();
       
        state_manager.setPath(path.poses);
        current_waypoint_idx_ = 0;

    }

    void global_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            Stamped3DVector global_position = Stamped3DVector(
                msg->header.stamp,
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z
            );
            state_manager.setGlobalBasePosition(global_position);
        
            auto& q_msg = msg->pose.pose.orientation;
            Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
            state_manager.setGlobalBaseOrientation(q);

            Stamped3DVector global_velocity = Stamped3DVector(
                msg->header.stamp,
                msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z
            );
            state_manager.setGlobalBaseVelocity(global_velocity);
        }

    void local_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Stamped3DVector local_position = Stamped3DVector(msg->header.stamp, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        state_manager.setLocalPosition(local_position);

        Stamped3DVector local_velocity = Stamped3DVector(msg->header.stamp, msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        state_manager.setLocalVelocity(local_velocity);
        

    }

    void aruco_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        aruco_last_seen_timer = this->now();
        Stamped3DVector aruco_position = Stamped3DVector(this->now(), msg->position.x, msg->position.y, msg->position.z);
        state_manager.setArucoPose(aruco_position);
        Eigen::Quaterniond aruco_orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        state_manager.setArucoOrientation(aruco_orientation);
    }
    
    // --- Publisher
    void publish_base_state()
    {
        interfaces::msg::BaseState msg;
        Eigen::Quaterniond global_orientation = state_manager.getGlobalBaseOrientation();
        Eigen::Vector3d euler_angles = transformation.quaternion_to_euler(global_orientation);
        msg.timestamp = this->now().seconds();
        msg.orientation = {
            euler_angles.x(),
            euler_angles.y(),
            euler_angles.z()
        };
        
        auto now = get_clock()->now();


        if (last_time.nanoseconds() == 0) {
            last_time = now;
        }
    
        double d_time = (now - last_time).seconds();
        last_time = now;

        double current_velocity = transformation.velocity_vector(state_manager.getGlobalBasePosition(), previous_position, d_time);
        msg.velocity_vector = current_velocity;

        base_state_pub->publish(msg);
        
    }

    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BaseCommandAction::Goal> goal)
    {
        static const std::vector<std::string> valid_commands = {"goto", "stop", "manual"};
        RCLCPP_INFO(this->get_logger(), "Received goal request with command: %s", goal->command.c_str());
        // Here you can add logic to accept or reject the goal based on its content
         if (std::find(valid_commands.begin(), valid_commands.end(), goal->command) == valid_commands.end())
        {
            RCLCPP_WARN(get_logger(), "Rejected goal with invalid command");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if(goal->command == "goto")
        {
            if (goal->target_pose.size() != 3)
            {
            RCLCPP_WARN(get_logger(), "Rejected 'goto' command with invalid target_pose size: %zu", goal->target_pose.size());
            return rclcpp_action::GoalResponse::REJECT;
            }

        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleBaseCommand> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleBaseCommand> goal_handle)
    {
        std::thread([this, goal_handle]()
                    { execute(goal_handle); })
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandleBaseCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<BaseCommandAction::Result>();
        
        try
        {
            if (goal->command == "goto")
            {
                RCLCPP_INFO(this->get_logger(), "Executing 'goto' command to target pose: [%.2f, %.2f, %.2f]", goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]);
                control_mode = 1;
                reached_target_angle = false;
                state_manager.setControlMode(control_mode);
                start_control_loop(goal, result);
                result->success = true;
                goal_handle->succeed(result);
            }
            else if (goal->command == "stop")
            {
                RCLCPP_INFO(this->get_logger(), "Executing 'stop' command");

                stop_robot();

                result->success = true;
                result->message = "Robot stopped";
                goal_handle->succeed(result);
            }
            else if (goal->command == "manual")
            {
                RCLCPP_INFO(this->get_logger(), "Executing 'manual' command");
                state_manager.setControlMode(4);
                current_goal = goal;
                if (!control_timer) {
                    control_timer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { control_loop(current_goal); }
                    );
                }
                result->success = true;
                goal_handle->succeed(result);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received unknown command: %s", goal->command.c_str());
                result->success = false;
                goal_handle->abort(result);
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while executing goal: %s", e.what());
            result->success = false;
            goal_handle->abort(result);
        }

    }


    void start_control_loop(const std::shared_ptr<const BaseCommandAction::Goal> goal, 
                                  const std::shared_ptr<BaseCommandAction::Result> result) 
    {
        current_waypoint_idx_ = 0; 
        reached_target_angle = false;
        
        Stamped3DVector target_position(
            rclcpp::Time(0, 0), // Timestamp can be set to zero or current time if needed
            goal->target_pose[0], 
            goal->target_pose[1], 
            goal->target_pose[2]);
        state_manager.setTargetPosition(target_position);
        state_manager.setGoalPosition(target_position);

        current_goal = goal;

        if(!control_timer){
            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(100), // Control loop runs every 100 ms
                [this]()
                {
                    control_loop(current_goal);
                }
            );
        }
          
    }
    




    void control_loop(const std::shared_ptr<const BaseCommandAction::Goal> goal)
    {
        auto now = get_clock()->now();


        if (last_time.nanoseconds() == 0) {
            last_time = now;
        }
    
        double d_time = (now - last_time).seconds();
        last_time = now;

        Stamped3DVector current_position = state_manager.getGlobalBasePosition();
        Eigen::Quaterniond current_orientation = state_manager.getGlobalBaseOrientation();
        Eigen::Vector3d euler_angles = transformation.quaternion_to_euler(current_orientation);
        Stamped3DVector target_position = state_manager.getTargetPosition();
        Stamped3DVector goal_position = state_manager.getGoalPosition();
        Stamped3DVector global_velocity = state_manager.getGlobalBaseVelocity();
        Stamped3DVector aruco_position = state_manager.getArucoPose();
        Eigen::Vector3d aruco_orientation = transformation.quaternion_to_euler(state_manager.getArucoOrientation());
        Stamped3DVector transformed_aruco = transformation.aruco_translation(aruco_position, aruco_orientation, current_position, euler_angles);
        std::cout << "Aruco Position: x=" << aruco_position.x() << ", y=" << aruco_position.y() << ", z=" << aruco_position.z() << std::endl;
        std::cout << "Transformed Aruco Position: x=" << transformed_aruco.x() << ", y=" << transformed_aruco.y() << ", z=" << transformed_aruco.z() << std::endl;
       
        switch (state_manager.getControlMode())
        {
        case 0: // idle
            // Do nothing
            break;
        case 1: // goto
            {
            //The turn controller
            if(!reached_target_angle){
                
                 //Another segmentation safety guard: wow
                path_ = state_manager.getPath();
                if (path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Path is empty, cannot execute goto command");
                    return;
                }

             
                
                //Looking at how close waypoints are, if under accepted distance, skip
                while (current_waypoint_idx_ < (int)path_.size() - 1) {
                    auto &wp = path_[current_waypoint_idx_];
                    Stamped3DVector wp_pos(wp.header.stamp, wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
                    if (controller.euclidean_distance(current_position, wp_pos) < accepted_distance.waypoint) {
                        current_waypoint_idx_++;
                    } else {
                        break;
                    }
                }


                auto &way_point = path_[current_waypoint_idx_];
                target_position = Stamped3DVector(way_point.header.stamp, 
                                                    way_point.pose.position.x, 
                                                    way_point.pose.position.y, 
                                                    way_point.pose.position.z);

                state_manager.setTargetPosition(target_position);
                
                TurnResult cmd_vel_angular = controller.turn_controller(
                    euler_angles, 
                    target_position,
                    current_position,
                    d_time, 
                    previous_angle_error);

                cmd_vel_angular.cmd.linear.x = 0.0;
                cmd_vel_pub->publish(cmd_vel_angular.cmd);
                
                if (std::abs(cmd_vel_angular.angle_error) < accepted_distance.angular){
                    std::cout << "Target angle reached, switching to linear control" << std::endl;
                    reached_target_angle = true;
                    previous_position_error.X.error = 0.0; // Reset position error for linear control
                    previous_angle_error.Z.error = 0.0; // Reset angle error for linear control
                    
                }
            }
            else{

                //Another segmentation safety guard:
                path_ = state_manager.getPath();
                if (path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Path is empty, cannot execute goto command");
                    return;
                }

             
                
                //Looking at how close waypoints are, if under accepted distance, skip
                while (current_waypoint_idx_ < (int)path_.size() - 1) {
                    auto &wp = path_[current_waypoint_idx_];
                    Stamped3DVector wp_pos(wp.header.stamp, wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
                    if (controller.euclidean_distance(current_position, wp_pos) < accepted_distance.waypoint) {
                        current_waypoint_idx_++;
                    } else {
                        break;
                    }
                }


                auto &way_point = path_[current_waypoint_idx_];
                target_position = Stamped3DVector(way_point.header.stamp, 
                                                    way_point.pose.position.x, 
                                                    way_point.pose.position.y, 
                                                    way_point.pose.position.z);

               
                //The linear controller
                geometry_msgs::msg::Twist cmd_vel = controller.dd_PD_controller_2(
                    current_position,
                    euler_angles,
                    target_position,
                    d_time,
                    previous_velocity_error,
                    previous_angle_error,
                    global_velocity
                );
                //geometry_msgs::msg::Twist cmd_vel = controller.dd_PD_controller(
                //    current_position,
                //    euler_angles,
                //    target_position,
                //    d_time,
                //    previous_position_error,
                //    previous_angle_error
                //);
                //std::cout << "Publishing cmd_vel: linear.x=" << cmd_vel.linear.x << ", angular.z=" << cmd_vel.angular.z << std::endl;
                cmd_vel_pub->publish(cmd_vel);
                           
                if(controller.euclidean_distance(current_position, goal_position) < accepted_distance.waypoint){
                    RCLCPP_INFO(this->get_logger(), "!!!!!!!!!Switch to precision mode!!!!!!!!!!");
                    state_manager.setControlMode(2); // Switch to precision mode
                    
                }

     
            }
            break;
            }
        case 2: 
        {
            // Final precision controller, looking only at goal
            geometry_msgs::msg::Twist cmd_vel = controller.dd_PD_precision_controller(
                    current_position,
                    euler_angles,
                    goal_position,
                    d_time,
                    previous_position_error,
                    previous_angle_error
                );
                cmd_vel_pub->publish(cmd_vel);
            if(controller.euclidean_distance(current_position, goal_position) < goal_distance.threshold){
                    RCLCPP_INFO(this->get_logger(), "Target position reached");
                  
                    if(this->now() - aruco_last_seen_timer < rclcpp::Duration(1s)){
                        std::cout << "Aruco marker seen recently, switching to aruco mode" << std::endl;
                        state_manager.setControlMode(3); // Switch to aruco mode

                    }
                    else{
                        std::cout << "Aruco marker not seen for a while, searching for marker" << std::endl;
                        cmd_vel.angular.z = 0.3; // Rotate in place to search for marker
                        cmd_vel_pub->publish(cmd_vel);
                    
                    }
                    }    
            break;
        }
        case 3: // Aruco Mode
            {
                geometry_msgs::msg::Twist cmd_vel;
                    
                std::cout << "IM RUNNING!!!!" << std::endl;

                cmd_vel.angular.z = 0.0; // Stop rotating in place to search for marker
                cmd_vel_pub->publish(cmd_vel);
                //geometry_msgs::msg::Twist cmd_vel= controller.od_PD_precision_controller(current_position, 
                //        euler_angles,
                //        aruco_position,
                //        d_time,
                //        previous_position_error,
                //        previous_angle_error
                //);
                
                //cmd_vel_pub->publish(cmd_vel);
                //if(controller.euclidean_distance(current_position, TBD) < goal_distance.threshold){
                //    RCLCPP_INFO(this->get_logger(), "Target position reached");
                //    cmd_vel.linear.x = 0.0;
                //    cmd_vel.angular.z = 0.0;
                //    Stamped3DVector target_profile(get_clock()->now(), 0.0, 0.0, 0.0);
                //    state_manager.setTargetPosition(target_profile);
                //    reached_target_angle = false;
                //    previous_position_error.X.error = 0.0;
                //    previous_angle_error.Z.error = 0.0;
                //    cmd_vel_pub->publish(cmd_vel);
                //    current_waypoint_idx_ = 0;
                //    stop_control_loop();
                //}
                break;
            }
            
        case 4: // manual control 
            {
                if (!keyboard_running_)
                    start_keyboard_thread();

                geometry_msgs::msg::Twist cmd_vel;
                const double lin_speed = 0.9;
                const double ang_speed = 1.0;

                if (key_up)    cmd_vel.linear.x  =  lin_speed;
                if (key_down)  cmd_vel.linear.x  = -lin_speed;
                if (key_left)  cmd_vel.angular.z =  ang_speed;
                if (key_right) cmd_vel.angular.z = -ang_speed;
                if (key_side_left)  cmd_vel.linear.y =  lin_speed; 
                if (key_side_right) cmd_vel.linear.y = -lin_speed; 

                cmd_vel_pub->publish(cmd_vel);
                
                break;
            }
        case 5: //Stop
            {
            stop_robot();
            break;
            }
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown control mode: %d", state_manager.getControlMode());
            break;
        };

    }

        void publish_zero_velocity()
    {
        geometry_msgs::msg::Twist cmd_vel;

        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        cmd_vel_pub->publish(cmd_vel);
    }


    void stop_robot()
    {
        RCLCPP_WARN(this->get_logger(), "Stopping robot");

        // Immediately send zero velocity
        publish_zero_velocity();

        // Reset target/controller state
        Stamped3DVector target_profile(get_clock()->now(), 0.0, 0.0, 0.0);
        state_manager.setTargetPosition(target_profile);
        state_manager.setGoalPosition(target_profile);

        reached_target_angle = false;
        current_waypoint_idx_ = 0;

        previous_position_error.X.error = 0.0;
        previous_angle_error.Z.error = 0.0;
        previous_velocity_error.dX.error = 0.0;

        // Stop the active control loop and return to idle
        stop_control_loop();

        // Send zero velocity again after stopping the timer
        publish_zero_velocity();

        RCLCPP_WARN(this->get_logger(), "Robot stopped");
    }




    void stop_control_loop()
    {
        if (control_timer) {
            control_timer->cancel();
            control_timer.reset();
            RCLCPP_INFO(this->get_logger(), "Control loop stopped");
        }
        state_manager.setControlMode(0);
        
    }
    void start_keyboard_thread()
    {
        keyboard_running_ = true;
        keyboard_thread_ = std::thread([this]()
        {
            int tty_fd = open("/dev/tty", O_RDWR);
            if (tty_fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "Cannot open /dev/tty for keyboard input");
                keyboard_running_ = false;
                return;
            }

            struct termios oldt, newt;
            tcgetattr(tty_fd, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(tty_fd, TCSANOW, &newt);
            fcntl(tty_fd, F_SETFL, O_NONBLOCK);

            RCLCPP_INFO(this->get_logger(), "Keyboard control active: W/A/S/D to move, Q to quit");

            while (keyboard_running_)
            {
                key_up = key_down = key_left = key_right = key_side_left = key_side_right = false;
                char c;
                while (read(tty_fd, &c, 1) > 0)
                {
                    switch (tolower(c))
                    {
                        case 'w': key_up    = true; break;
                        case 's': key_down  = true; break;
                        case 'a': key_left  = true; break;
                        case 'd': key_right = true; break;
                        case 'q': key_side_left = true; break; 
                        case 'e': key_side_right = true; break; 
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            tcsetattr(tty_fd, TCSANOW, &oldt);
            close(tty_fd);
        });
    }

    void stop_keyboard_thread()
    {
        keyboard_running_ = false;
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
    }
    

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabBaseNode>());
    rclcpp::shutdown();
    return 0;
}