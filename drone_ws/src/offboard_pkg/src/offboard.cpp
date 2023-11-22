#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/qos.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>


#define DISTANCE 10
#define DISERROR 1

mavros_msgs::msg::State current_state;
void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
	current_state = *msg;
}

geometry_msgs::msg::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
	local_pos = *msg;
	RCLCPP_INFO(rclcpp::get_logger("local_pos_cb"),"x = %f,y= %f,z= %f",local_pos.pose.position.x,local_pos.pose.position.y,local_pos.pose.position.z);	
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("offboard_node");

    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
	auto local_pos_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>
		("mavros/local_position/pose",  qos, local_pos_cb);

	auto state_sub = node->create_subscription<mavros_msgs::msg::State>
		("mavros/state", 10, state_cb);

	auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>
		("mavros/setpoint_position/local", 10);

	auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>
		("mavros/cmd/arming");
	auto set_mode_client = node->create_client<mavros_msgs::srv::SetMode>
		("mavros/set_mode");

	rclcpp::Rate rate(20.0);

	while (rclcpp::ok() && !current_state.connected) {
		RCLCPP_INFO(node->get_logger(),"unconnected");
		rclcpp::spin_some(node);
		rate.sleep();
	}
	
	geometry_msgs::msg::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;

	for (int i = 100; rclcpp::ok() && i > 0; --i) {
		local_pos_pub->publish(pose);
		rclcpp::spin_some(node);
		rate.sleep();
	}

    auto setmode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    setmode_request->custom_mode = "OFFBOARD";

    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

	rclcpp::Time last_request = rclcpp::Clock().now();


	int step = 0;
	int sametimes = 0;

	while (rclcpp::ok()) {
		if (current_state.mode != "OFFBOARD" && (rclcpp::Clock().now() - last_request > rclcpp::Duration(5,0))) 
        {
			auto setmode_response = set_mode_client->async_send_request(setmode_request);
            if (rclcpp::spin_until_future_complete(node, setmode_response) == rclcpp::FutureReturnCode::SUCCESS) 
            {
				if(setmode_response.get()->mode_sent) RCLCPP_INFO(node->get_logger(),"Offboard enabled");
                else RCLCPP_INFO(node->get_logger(),"Failed to set offboard");
			}
			last_request = rclcpp::Clock().now();
		}
		else 
        {
			if (!current_state.armed && (rclcpp::Clock().now() - last_request > rclcpp::Duration(5,0))) 
            {
				auto arm_response = arming_client->async_send_request(arm_request);
                if (rclcpp::spin_until_future_complete(node, arm_response) == rclcpp::FutureReturnCode::SUCCESS) 
                {
					if (arm_response.get()->success) RCLCPP_INFO(node->get_logger(),"Vehicle armed");
                    else RCLCPP_INFO(node->get_logger(),"Failed to arm");
				}
				last_request = rclcpp::Clock().now();
			}
			else
			{
				switch (step)
				{
				case 0:  // first point  0  0  10
					//take off to 2m
					pose.pose.position.x = 0;
					pose.pose.position.y = 0;
					pose.pose.position.z = DISTANCE;
					//dont care x and y ,only need z = distance
					if (local_pos.pose.position.z > (DISTANCE-DISERROR) && local_pos.pose.position.z < (DISTANCE+DISERROR))
					{
						if (sametimes > 100)
						{
							
							sametimes = 0;
							step = 1;
							pose.pose.position.x = DISTANCE;
							pose.pose.position.y = 0;
							pose.pose.position.z = DISTANCE;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub->publish(pose);
					break;
				case 1: //seconde  10  0 10 
					
					if (local_pos.pose.position.x > (DISTANCE -DISERROR) && local_pos.pose.position.x < (DISTANCE + DISERROR))
					{
						if (sametimes > 100)
						{
							
							step = 2;
							pose.pose.position.x = DISTANCE;
							pose.pose.position.y = DISTANCE;
							pose.pose.position.z = DISTANCE;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub->publish(pose);
					break;
				case 2:   //10 10  10
					
					if (local_pos.pose.position.y > (DISTANCE - DISERROR) && local_pos.pose.position.y < (DISTANCE +DISERROR))
					{
						if (sametimes > 100)
						{
							
							step = 3;
							pose.pose.position.x = 0;
							pose.pose.position.y = DISTANCE;
							pose.pose.position.z = DISTANCE;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub->publish(pose);
					break;
				case 3: // 0 10 10
					
					if (local_pos.pose.position.x > -DISERROR && local_pos.pose.position.x < DISERROR)
					{
						if (sametimes > 100)
						{
							
							step = 4;
							pose.pose.position.x = 0;
							pose.pose.position.y = 0;
							pose.pose.position.z = DISTANCE;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					local_pos_pub->publish(pose);
					break;
				case 4:  // 0 0 10
					
					if (local_pos.pose.position.y > -DISERROR && local_pos.pose.position.y < DISERROR)// position ok
					{
						if (sametimes > 100)
						{
							step = 0;  //return loop fly to 10 0 10 
						}
						else
							sametimes++;
					}
					else//position not succes 
					{
						sametimes = 0;
					}
					local_pos_pub->publish(pose);
					break;
				case 5:
                    setmode_request->custom_mode = "AUTO.LAND";
					if (current_state.mode != "AUTO.LAND" && (rclcpp::Clock().now() - last_request > rclcpp::Duration(5,0)))
					{
                        auto setmode_response = set_mode_client->async_send_request(setmode_request);
						if (rclcpp::spin_until_future_complete(node, setmode_response) == rclcpp::FutureReturnCode::SUCCESS)
						{
							if (setmode_response.get()->mode_sent) RCLCPP_INFO(node->get_logger(),"AUTO.LAND enabled");
                            else RCLCPP_INFO(node->get_logger(),"Failed to set AUTO.LAND");
						}
						last_request = rclcpp::Clock().now();
					}
					break;
				default:
					break;
				}
			}
		}



		rclcpp::spin_some(node);
		rate.sleep();
	}

	return 0;
}