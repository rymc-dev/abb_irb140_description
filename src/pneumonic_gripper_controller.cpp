int main() { return 0; }
// #include "rclcpp/rclcpp.hpp"

// #include <string>
// #include <vector>

// #include "std_msgs/msg/JointState"
// #include "std_srvs/srv/Trigger"


// #include <control_msgs/action/gripper_command.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>


// class CustomGripperServer : public rclcpp::Node {
// private:
// 	using GripperAction = control_msgs::action::GripperCommand;
// 	rclcpp_action::Server<GripperAction>::SharedPtr server_;

// private: 
// 	rclcpp::ActionServer <>action_server_();

// 	void action_request() {

// 	}

// 	void action_feedback(){

// 	}

// 	void action_goal_complete() {

// 	}
// }

// class PneumaticGripperController : public rclcpp::Node {
// public: 
// 	PneumaticGripperController(const std::vector<std::string>& joint_names) 
// 	: joint_names_(joint_names),
// 	Node("PneumaticGripperControllerNode") 
// 	{
// 		RCLCPP_INFO(this->get_logger(), "Pneumatic Gripper Controller Node Intialized!");
// 		joint_pub_ = this->create_publisher<std_msgs::msg::JointState>("joint_states", 10);
// 		toggle_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(
// 			"toggle_gripper",
// 			&toggle_gripper_
// 		)
// 	}

// `

// private:
// 	std::vector<std::string> joint_names_{};
// 	rclcpp::Publisher<std_msgs::msg::JointState>::SharedPtr joint_pub_ {};
// 	rclcpp::Service <std_srvs::srv::Trigger>::SharedPtr toggle_gripper_srv_ {}; 

// 	void toggle_gripper_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
// 						std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
// 	{

		
// 	}

// };


// int main(int argc, char * argv[]) 
// {
// 	rclcpp::init(argc, argv);

// 	auto node = std::make_shared<PneumaticGripperController>();
// 	rclcpp::spin(node);

// 	rclcpp::shutdown();
// 	return 0;
// }