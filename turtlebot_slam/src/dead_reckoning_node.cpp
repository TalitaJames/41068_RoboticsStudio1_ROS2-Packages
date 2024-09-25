#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/Odometry>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.h>

#include <cmath>



class DeadReckoningNode : public rclcpp::Node{
public:
	DeadReckoningNode() : Node("dead_reckoning_node"), gen_(rd_()), nois
	{
		

		odom_noisy_pub_ = this->create_publisher<nav:msgs::msg::Odometry>("odom_noisy",10);
		odom_sub_ = this->create_subscription<nav:msgs::msg::Odometry>(
			"odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1));
	}




}

