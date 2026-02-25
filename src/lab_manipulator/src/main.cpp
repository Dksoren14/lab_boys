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

        std::cout << "Starting server.... " << std::endl;


        sub_state = create_subscription<interfaces::msg::SocketMsg>(
            "/lab/lab_chassis/out/socket_msg", qos,
            [this](const interfaces::msg::SocketMsg::SharedPtr msg)
            { messageCallback(msg); }
            );

      


    }
private:

    void messageCallback(const interfaces::msg::SocketMsg::SharedPtr msg)
    {
        timestamp = msg->timestamp;
        data = msg->data;
    }

    rclcpp::Subscription<interfaces::msg::SocketMsg>::SharedPtr sub_state;

    float timestamp;
    string data;

    void receiveMessage()
    {
        // creating socket
        int clientSocket = socket(AF_INET, SOCK_STREAM, 0);

        // specifying address
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(8080); //converting to network byte order
        serverAddress.sin_addr.s_addr = INADDR_ANY;

        // sending connection request
        connect(clientSocket, (struct sockaddr*)&serverAddress,
                sizeof(serverAddress));

        // sending data
        const char* message = "Hello, server!";
        send(clientSocket, message, strlen(message), 0);

        // closing socket
        close(clientSocket);
    }
    

};
int main(int argc, char * argv[]) {
    
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<LabManipulatorNode>());  
    rclcpp::shutdown();
    return 0;
}