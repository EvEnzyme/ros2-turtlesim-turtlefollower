#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TurtleVeloPublisher : public rclcpp::Node
{
	public:
		TurtleVeloPublisher()
		: Node("turtle_control")
    	{
			auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
			param_desc.description = "velocity control";

			// create the parameter, name = "velocity", defualt value = 3.0
     		this->declare_parameter("velocity", 3.0, param_desc);
			velo_param = this->get_parameter("velocity").as_double();

			// create publisher, topic = "vel_topic"
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    		timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleVeloPublisher::timer_callback, this));
		}

		void timer_callback()
		{
			auto twist = geometry_msgs::msg::Twist();
			twist.linear.x = 1.0 * velo_param;
			twist.angular.z = 1.0;

			publisher_-> publish(twist);
		}

	private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		double velo_param;
	
	};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleVeloPublisher>());
  rclcpp::shutdown();
  return 0;
}