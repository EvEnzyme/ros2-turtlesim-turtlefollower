#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;	// placeholder for callback function binding

class TurtleFollower : public rclcpp::Node
{
public:
	TurtleFollower()
		: Node("turtle_follower")
	{
		/*
		our follower turtle subscribes to two topics:
			1. the position of the leader turtle
			2. the position of itself
		
		it publishes to one topic:
			1. its velocity command, for controlling what direction and how fast it shoud go next

		*/

		pose_subscriber_turtle1_ = this->create_subscription<turtlesim::msg::Pose>(
			"/turtle1/pose", 10, std::bind(&TurtleFollower::leader_pose_callback, this, _1));

		pose_subscriber_turtle2_ = this->create_subscription<turtlesim::msg::Pose>(
			"/turtle2/pose", 10, std::bind(&TurtleFollower::follower_pose_callback, this, _1));

		turlle2_velo_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
	}

private:
	void leader_pose_callback(const turtlesim::msg::Pose::SharedPtr leader_msg)
	{
		

	}


	void follower_pose_callback(const turtlesim::msg::Pose::SharedPtr follower_msg)
	{

	}

	double follower_current_x = 0.0;
	double follower_current_y = 0.0;
	double follower_current_theta = 0.0;
	double threshold = 1.0;
	
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_turtle1_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_turtle2_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turlle2_velo_publisher_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurtleFollower>());
	rclcpp::shutdown();
	return 0;
}