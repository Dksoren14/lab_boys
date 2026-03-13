#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

#include "interfaces/msg/socket_msg.hpp"

using namespace std;



class LabManipulatorNode : public rclcpp::Node
{
public:
    LabManipulatorNode() 
    : Node("lab_manipulator_node")
    {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        std::cout << "Starting client.... " << std::endl;


        sub_state = create_subscription<interfaces::msg::SocketMsg>(
            "/lab/lab_chassis/in/socket_msg", qos,
            [this](const interfaces::msg::SocketMsg::SharedPtr msg)
            { messageCallback(msg); }
            );

            

    }
private:

    void messageCallback(const interfaces::msg::SocketMsg::SharedPtr msg)
{
    timestamp = msg->timestamp;
    data = msg->data;

    //std::cout << "Received message: "
    //          << msg->data
    //          << " at time: "
    //          << std::setprecision(10) << msg->timestamp
    //          << std::endl;
}

    rclcpp::Subscription<interfaces::msg::SocketMsg>::SharedPtr sub_state;

    float timestamp;
    string data;

   
    

};
int main(int argc, char * argv[]) {
    
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabManipulatorNode>());  
    rclcpp::shutdown();
    return 0;
}