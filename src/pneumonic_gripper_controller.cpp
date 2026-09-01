#include "rclcpp/rclcpp.hpp"

class PneumaticGripperController : public rclcpp::Node {
public: 
	PneumaticGripperController() : Node("PneumaticGripperControllerNode") 
	{
		RCLCPP_INFO(this->get_logger(), "Hello ROS2, This node has started!");
	}
};


int main(int argc, char * argv[]) 
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<PneumaticGripperController>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}