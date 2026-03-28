
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
#include "geometry_msgs/msg/pose_array.hpp"

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
    LabBaseNode() 
    : Node("lab_base_node"),
    state_manager(),
    controller(state_manager)
    {
        std::cout << "LabBaseNode initialized" << std::endl;
        PIDControllerGains pid_gains;
        this->declare_parameter<double>("controller.kp", 1.0);
        this->declare_parameter<double>("controller.kd", 0.1);

        pid_gains.kp = this->get_parameter("controller.kp").as_double();
        pid_gains.kd = this->get_parameter("controller.kd").as_double();

        std::cout << "Controller gains: kp=" << pid_gains.kp << ", kd=" << pid_gains.kd << std::endl;

        std::cout << R"(

                               /T /I                       
                              / |/ | .-~/                  
                          T\ Y  I  |/  /  _          
         /T               | \I  |  I  Y.-~/                 
        I l   /I       T\ |  |  l  |  T  /                  
     T\ |  \ Y l  /T   | \I  l   \ `  l Y                       
 __  | \l   \l  \I l __l  l   \   `  _. |                   
 \ ~-l  `\   `\  \  \\ ~\  \   `. .-~   |                    
  \   ~-. "-.  `  \  ^._ ^. "-.  /  \   |                   
.--~-._  ~-  `  _  ~-_.-"-." ._ /._ ." ./                   
 >--.  ~-.   ._  ~>-"    "\   7   7   ]                    
^.___~"--._    ~-{  .-~ .  `\ Y . /    |                    
 <__ ~"-.  ~       /_/   \   \I  Y   : |                    
   ^-.__           ~(_/   \   >._:   | l______              
       ^--.,___.-~"  /_/   !  `-.~"--l_ /     ~"-.          
              (_/ .  ~(   /'     "~"--,Y   -=b-. _)             
               (_/ .  \  :           / l      c"~o  \
                \ /    `.    .     .^   \_.-~"~--.  )           
                 (_/ .   `  /     /       !       )/        
                  / / _.   '.   .':      /        '         
                  ~(_/ .   /    _  `  .-<_                  
                    /_/ . ' .-~" `.  / \  \          ,z=.       
                    ~( /   '  :   | K   "-.~-.______//          
                      "-,.    l   I/ \_    __{--->._(==.        
                       //(     \  <    ~"~"     //          
                      /' /\     \  \     ,v=.  ((           
                    .^. / /\     "  }__ //===-  `           
                   / / ' '  "-.,__ {---(==-                 
                 .^ '       :  T  ~"   ll
                / .  .  . : | :!        \\
               (_/  /   | | j-"          ~^             
                 ~-<_(_.^-~"                            

)" << std::endl;
        controller.setGains(pid_gains);
        // --- Quality of Service settings for subscriptions ---
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);


        // --- Subscriptions ---
      //  local_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      //      "/model/r100/odometry",
      //      qos,
      //      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
      //          local_callback(msg);
      //      }
      //  );

        base_global_position_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/world/car_world/dynamic_pose/info",
            qos,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg){
                global_callback(msg);
            }
        );


        // --- Publishers ---
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
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
    Transformation transformation;

    rclcpp::TimerBase::SharedPtr control_timer;
    PositionError previous_position_error;
    PositionError previous_angle_error;
    std::shared_ptr<const BaseCommandAction::Goal> current_goal;
    std::shared_ptr<BaseCommandAction::Result> current_result;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr base_global_position_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;  
    rclcpp::Time last_time;
    bool reached_target_angle = false;


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

        void global_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        Stamped3DVector global_position = Stamped3DVector(msg->header.stamp, msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);
        Stamped3DVector global_orientation = Stamped3DVector(msg->header.stamp, msg->poses[0].orientation.x, msg->poses[0].orientation.y, msg->poses[0].orientation.z); 
        state_manager.setGlobalBasePosition(global_position);
        auto& q_msg = msg->poses[0].orientation;

        Eigen::Quaterniond q(
            q_msg.w,
            q_msg.x,
            q_msg.y,
            q_msg.z
        );

        state_manager.setGlobalBaseOrientation(q);

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
                reached_target_angle = false;
                state_manager.setControlMode(control_mode);
                start_control_loop(goal, result);
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


    void start_control_loop(const std::shared_ptr<const BaseCommandAction::Goal> goal, 
                                  const std::shared_ptr<BaseCommandAction::Result> result) 
    {
        
        Stamped3DVector target_position(
            rclcpp::Time(0, 0), // Timestamp can be set to zero or current time if needed
            goal->target_pose[0], 
            goal->target_pose[1], 
            goal->target_pose[2]);
        state_manager.setTargetPosition(target_position);

        current_goal = goal;
        current_result = result;

        if(!control_timer){
            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(100), // Control loop runs every 100 ms
                [this]()
                {
                    control_loop(current_goal, current_result);
                }
            );
        }
          
    }
    




    void control_loop(const std::shared_ptr<const BaseCommandAction::Goal> goal, const std::shared_ptr<BaseCommandAction::Result> result)
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
       
        switch (state_manager.getControlMode())
        {
        case 0: // idle
            // Do nothing
            break;
        case 1: // goto
            {


            
            
            if(!reached_target_angle){
                
                TurnResult cmd_vel_angular = controller.turn_controller(
                    euler_angles, 
                    target_position,
                    current_position,
                    d_time, 
                    previous_angle_error);
                    
                cmd_vel_pub->publish(cmd_vel_angular.cmd);
                if (std::abs(cmd_vel_angular.angle_error) < 0.04) {
                    std::cout << "Target angle reached, switching to linear control" << std::endl;
                    reached_target_angle = true;
                    previous_position_error.X.error = 0.0; // Reset position error for linear control
                    previous_angle_error.Z.error = 0.0; // Reset angle error for linear control
                    
                }
            }
            else{
                geometry_msgs::msg::Twist cmd_vel = controller.dd_PD_controller(
                    current_position,
                    euler_angles,
                    target_position,
                    d_time,
                    previous_position_error,
                    previous_angle_error
                );
                cmd_vel_pub->publish(cmd_vel);
                std::cout << "How close to target: " << controller.euclidean_distance(current_position, target_position) << std::endl;
                if(controller.euclidean_distance(current_position, target_position) < 0.03){
                    RCLCPP_INFO(this->get_logger(), "Target position reached");
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    Stamped3DVector target_profile(get_clock()->now(), 0.0, 0.0, 0.0);
                    state_manager.setTargetPosition(target_profile);
                    reached_target_angle = false;
                    previous_position_error.X.error = 0.0;
                    previous_angle_error.Z.error = 0.0;
                    cmd_vel_pub->publish(cmd_vel);
                    result->success = true;
                    stop_control_loop();
                }
            }
            
                
            break;
            }
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
        if (control_timer) {
            control_timer->cancel();
            control_timer.reset();
            RCLCPP_INFO(this->get_logger(), "Control loop stopped");
        }
        state_manager.setControlMode(0);
        
    }
    

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabBaseNode>());
    rclcpp::shutdown();
    return 0;
}