#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ctime>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

// Include message types
#include "interfaces/msg/socket_msg.hpp"
#include "interfaces/action/base_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Include own headers
#include "state_manager.h"
#include "controller.h"

using namespace std;


class LabBaseNode : public rclcpp::Node
{
public:
    using BaseCommandAction = interfaces::action::BaseCommand;
    using GoalHandleBaseCommand = rclcpp_action::ServerGoalHandle<BaseCommandAction>;
    LabBaseNode() 
    : Node("lab_base_node"),
    state_manager(),
    controller(state_manager)
    {
        std::cout << "LabBaseNode initialized" << std::endl;
        // --- Quality of Service settings for subscriptions ---
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);


        // --- Subscriptions ---
        local_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/r100/odometry",
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                local_callback(msg);
            }
        );

        // --- Publishers ---
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/model/r100/cmd_vel", 10);

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
    }


private:
    StateManager state_manager;
    Controller controller;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  
    rclcpp::TimerBase::SharedPtr test_timer;
    time_t current_time;
    bool are_we_there_yet = false;
   
    rclcpp_action::Server<BaseCommandAction>::SharedPtr base_command_server;

    int control_mode = 0; // 0: idle, 1: goto, 2: stop

    void local_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Stamped3DVector local_position = Stamped3DVector(msg->header.stamp, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        state_manager.setLocalPosition(local_position);

        Stamped3DVector local_velocity = Stamped3DVector(msg->header.stamp, msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        state_manager.setLocalVelocity(local_velocity);
        //RCLCPP_INFO(this->get_logger(), "StateManager address: %p", &state_manager);
        //test_position();

    }

     //Just to test, will be removed later (maybe)
    void test_position(){
        Stamped3DVector local_position = state_manager.getLocalPosition();
        RCLCPP_INFO(this->get_logger(), "Current local position: x=%.2f, y=%.2f, z=%.2f", local_position.x(), local_position.y(), local_position.z());
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BaseCommandAction::Goal> goal)
    {
        static const std::vector<std::string> valid_commands = {"goto", "stop"};
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
                state_manager.setControlMode(control_mode);
                control_loop(goal, result);
            }
            else if (goal->command == "stop")
            {
                RCLCPP_INFO(this->get_logger(), "Executing 'stop' command");
                //execute_stop_command(result);
                result ->success = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received unknown command: %s", goal->command.c_str());
                result->success = false;
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while executing goal: %s", e.what());
            result->success = false;
        }

    }


    void execute_velocity_command(const std::shared_ptr<const BaseCommandAction::Goal> goal, const std::shared_ptr<BaseCommandAction::Result> result)
    {
       
        Stamped3DVector target_position(
            rclcpp::Time(0, 0), // Timestamp can be set to zero or current time if needed
            goal->target_pose[0], 
            goal->target_pose[1], 
            goal->target_pose[2]);
        state_manager.setTargetPosition(target_position);
        
        while (rclcpp::ok())
        {
            Stamped3DVector current_position = state_manager.getLocalPosition();
            RCLCPP_INFO(this->get_logger(), "Current position: x=%.2f, y=%.2f, z=%.2f", current_position.x(), current_position.y(), current_position.z());

            if (controller.simple_distance_test(current_position, target_position))
            {
                RCLCPP_INFO(this->get_logger(), "Reached target");
                geometry_msgs::msg::Twist zero_velocity;
                zero_velocity.linear.x = 0.0;
                zero_velocity.linear.y = 0.0;
                zero_velocity.linear.z = 0.0;
                cmd_vel_pub_->publish(zero_velocity);

                result->success = true;
                result->message = "Target position reached successfully";
                stop_control_loop();
                break;
            }
            Stamped3DVector local_velocity = state_manager.getLocalVelocity();
            geometry_msgs::msg::Twist cmd_vel = controller.simple_controller(current_position, target_position, local_velocity);
            RCLCPP_INFO(this->get_logger(), "Publishing velocity: %.2f", cmd_vel.linear.x);
            cmd_vel_pub_->publish(cmd_vel);


            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }




    void control_loop(const std::shared_ptr<const BaseCommandAction::Goal> goal, const std::shared_ptr<BaseCommandAction::Result> result)
    {
        switch (state_manager.getControlMode())
        {
        case 0: // idle
            // Do nothing
            break;
        case 1: // goto
            execute_velocity_command(goal, result);
            break;
        case 2: // stop
            //execute_stop_command(result);
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown control mode: %d", state_manager.getControlMode());
            break;
        };

    }

    void stop_control_loop()
    {
        state_manager.setControlMode(0);
        state_manager.setTargetPosition(state_manager.getLocalPosition());
        
        RCLCPP_INFO(this->get_logger(), "Control loop stopped");
    }
    

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabBaseNode>());
    rclcpp::shutdown();
    return 0;
}