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
#include "nav_msgs/msg/odometry.hpp"

// Include own headers
#include "state_manager.h"

using namespace std;


class LabBaseNode : public rclcpp::Node
{
public:
    LabBaseNode() 
    : Node("lab_base_node")
    {
    
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        local_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/r100/odometry",
            qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                local_position_callback(msg);
            }
        );
    }


private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;
    rclcpp::TimerBase::SharedPtr test_timer;
    time_t current_time;
    StateManager state_manager;

    void local_position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Stamped3DVector local_position = Stamped3DVector(msg->header.stamp, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        state_manager.setLocalPosition(local_position);
        

    }

    // Just to test, will be removed later (maybe)
    //void test_position(){
    //    Stamped3DVector local_position = state_manager.getLocalPosition();
    //    RCLCPP_INFO(this->get_logger(), "Current local position: x=%.2f, y=%.2f, z=%.2f", local_position.x(), local_position.y(), local_position.z());
    //}


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabBaseNode>());
    rclcpp::shutdown();
    return 0;
}