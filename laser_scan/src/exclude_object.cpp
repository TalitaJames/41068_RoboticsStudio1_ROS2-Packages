/**
 * @file exclude_object.cpp
 * @author Talita James (account@talitajames.com)
 * @brief Scans a laser message to remove cylinders of a fixed diameter
 * @version 0.1
 * @date 2024-10-02
 * 
 */

#include "exclude_object.h"

/**
 * @brief Construct a new Exclude Object:: Exclude Object object
 * 
 */
ExcludeObject::ExcludeObject() : Node("laser_scan_processor"){
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // subscribe to a publisher
        "/scan", 10, std::bind(&ExcludeObject::scanCallback, this, std::placeholders::_1));

    // Publishers
    scan_excluded_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_excluded_pub", 10);
    scan_shape_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_shape_pub", 10);
}

/**
 * @brief a repitious method called to gather subscriber data and publish new data
 * 
 * @param scan the laser scan message
 */
void ExcludeObject::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    int nthIncrement = 1;
    size_t totalNumOfScans = scan -> ranges.size();


    // Make a new message
    auto scan_excluded = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
    scan_excluded->angle_min = scan->angle_min;
    scan_excluded->angle_max = scan->angle_max;

    auto cartesianPoints = radialToCartesian(scan->ranges, scan->angle_max, scan->angle_min);
    // scan_excluded->ranges = std::vector<float>();

    float cylinderDiameter = 0.3; // [m] diameter of cylinder to exclude

    float previousRange = 0;
    for (size_t i = 0; i < totalNumOfScans; i++) {
        float currentRange = scan->ranges[i];
        if( std::abs(previousRange - currentRange)){
        /* code */

        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Ran callback");
    
    scan_excluded_pub->publish(*scan_excluded);
}


/**
 * @brief converts radial lidar data to cartedian data
 * 
 * @param lidarRanges a vector of ranges gathered [m]
 * @param startAngle the angle at which the scan starts [rad]
 * @param endAngle final angle of the scan [rad]
 * @return std::vector<std::pair<float, float>>, the coordinates in <x,y> form, local to the scan
 */
std::vector<std::pair<float, float>> radialToCartesian(const std::vector<float>& lidarRanges, float startAngle, float endAngle) {
    int numOfData = lidarRanges.size();
    std::vector<std::pair<float, float>> cartesian(numOfData); // x,y

    for (int i = 0; i < numOfData; ++i) {
        float theta = startAngle + (endAngle - startAngle) * i / (numOfData - 1); //note! already in radians
        float x = lidarRanges[i] * std::cos(theta);
        float y = lidarRanges[i] * std::sin(theta);
        cartesian[i] = std::make_pair(x, y);
    }

    return cartesian;
}

/**
 * @brief starts an "exclude object"
 * 
 * @param argc
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[]) {

    // Example usage
    // std::vector<float> lidar = {1.0, 2.0, 3.0, 4.0};
    // float startAngle = 0;
    // float endAngle = 1.5708;

    // std::vector<std::pair<float, float>> cartesian = radialToCartesian(lidar, startAngle, endAngle);
    // for (const auto& point : cartesian) {
    //     std::cout << "x: " << point.first << ", y: " << point.second << std::endl;
    // }

    return 0;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcludeObject>());
    rclcpp::shutdown();
    return 0;
}