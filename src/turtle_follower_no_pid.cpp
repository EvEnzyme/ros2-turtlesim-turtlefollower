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
		std::cout << "leader_pose_callback" << std::endl;
		// position of the leader turtle, turtle1
		double lead_turtle_x = leader_msg->x;
		double lead_turtle_y = leader_msg->y;

		auto twist = geometry_msgs::msg::Twist();

		// calculate the difference in position between turtle1 and turtle2
		double x_diff = lead_turtle_x - follower_current_x;
		double y_diff = lead_turtle_y - follower_current_y;

		// based on the difference in position, calcualte Euclidean distance
		double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff);

		// no pid implementation so turtle2 simply nodestops if it's too close to turtle1
		if (distance < threshold)
		{
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			std::cout << "reached distance limit, stop moving" << std::endl;
			turlle2_velo_publisher_->publish(twist);

			return;
		}

		// set linear velocity, cap at 2.0
		twist.linear.x = std::min(2.0, distance);

		// calculate the angle between turtle1 and turtle2
		double tangent_angle = std::atan2(y_diff, x_diff);
		double angle_diff = tangent_angle - follower_current_theta;

		// Normalize the angle to be between -π and π
		while (angle_diff > M_PI)
			angle_diff -= 2 * M_PI;
		while (angle_diff < -M_PI)
			angle_diff += 2 * M_PI;

		// Set angular velocity to turn towards turtle1, clamped within [-2.0, 2.0] rad/s
		twist.angular.z = std::clamp(angle_diff, -2.0, 2.0);

		turlle2_velo_publisher_->publish(twist);
	}
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_turtle1_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_turtle2_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turlle2_velo_publisher_;

	void follower_pose_callback(const turtlesim::msg::Pose::SharedPtr follower_msg)
	{
		follower_current_x = follower_msg->x;
		follower_current_y = follower_msg->y;
		follower_current_theta = follower_msg->theta;
	}

	double follower_current_x = 1.0;
	double follower_current_y = 1.0;
	double follower_current_theta = 0.0;
	double threshold = 1.0;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurtleFollower>());
	rclcpp::shutdown();
	return 0;
}