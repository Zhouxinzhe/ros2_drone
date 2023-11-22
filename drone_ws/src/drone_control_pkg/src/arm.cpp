#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp"
#include<chrono>

mavros_msgs::msg::State current_state;
void get_state(const mavros_msgs::msg::State::SharedPtr state)
{
    current_state = *state;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arm_node");
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, get_state);
    auto arm_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;
    

    // check whether the drone is connected
    while(rclcpp::ok() && !current_state.connected)
    {
        RCLCPP_INFO(node->get_logger(),"Drone isn't connected!");
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(node->get_logger(),"Drone connected!");

    // check whether the service is available
    while(rclcpp::ok() && !arm_client->service_is_ready())
    {
        RCLCPP_INFO(node->get_logger(),"Arm_service isn't available!");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(node->get_logger(),"Arm_service available!");

    // call the service
    auto response = arm_client->async_send_request(arm_request);
    if(rclcpp::spin_until_future_complete(node, response) == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if(response.get()->success) 
        {
            RCLCPP_INFO(node->get_logger(), "Drone armed!");
        } 
        else 
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm drone!");
        }
    } 
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed");
    }

    rclcpp::shutdown();
    return 0;
}