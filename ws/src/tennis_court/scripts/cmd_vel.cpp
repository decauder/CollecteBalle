#include <chrono>
#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;


class cmd_vel : public rclcpp::Node {
  	public:
		cmd_vel() : Node("Velocity_command") {
	  		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 10);
	  		timer_ = this->create_wall_timer(50ms, std::bind(&cmd_vel::timer_callback, this));
	}

  	private:
		void timer_callback() {
			geometry_msgs::msg::Twist msg;
	  		publisher_->publish(msg);
	}
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmd_vel>());
  rclcpp::shutdown();
  return 0;
}
