/**
 * @file exclude_object.h
 * @author Talita James (account@talitajames.com)
 * @brief Header for exclude_object
 * @version 0.1
 * @date 2024-10-02
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ExcludeObject : public rclcpp::Node {
public:
    ExcludeObject();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_excluded_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_shape_pub;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};


std::vector<std::pair<float, float>> radialToCartesian(const std::vector<float>& lidarRanges, float startAngle, float endAngle);