#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

#include "interfaces/msg/socket_msg.hpp"

using namespace std;


class LabBaseNode : public rclcpp::Node
{
public:
    LabBaseNode() 
    : Node("lab_base_node")
    {
    
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
         std::cout << "Starting server.... " << std::endl;



         test_publisher = create_publisher<interfaces::msg::SocketMsg>(
            "/lab/lab_chassis/in/socket_msg", qos
         );
        
            // Start a thread to send messages
        test_timer = create_wall_timer(1s, std::bind(&LabBaseNode::sendMessage, this)); // Send a message every 200 milliseconds
    }


private:
    rclcpp::Publisher<interfaces::msg::SocketMsg>::SharedPtr test_publisher;
    rclcpp::TimerBase::SharedPtr test_timer;
    time_t current_time;

    void sendMessage()
    {
        
        interfaces::msg::SocketMsg message;
        message.timestamp = this->get_clock()->now().seconds();
        message.data = "Hello, manipulator!";
        test_publisher->publish(message);

        
  
    }
};

int main(int argc, char * argv[])
{
    //// creating socket
    //int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
//
    //// specifying address
    //sockaddr_in serverAddress;
    //serverAddress.sin_family = AF_INET;
    //serverAddress.sin_port = htons(8080); //converting to network byte order
    //serverAddress.sin_addr.s_addr = INADDR_ANY;
//
    //// sending connection request
    //connect(clientSocket, (struct sockaddr*)&serverAddress,
    //        sizeof(serverAddress));
//
    //// sending data
    //const char* message = "Hello, server!";
    //send(clientSocket, message, strlen(message), 0);
//
    //// closing socket
    //close(clientSocket);

    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabBaseNode>());
    rclcpp::shutdown();
    return 0;
}