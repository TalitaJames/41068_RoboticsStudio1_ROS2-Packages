/**
 * @file nth_scan.cpp
 * @author Talita James (account@talitajames.com)
 * @brief Takes a laser scan, filters it to include only every nth data point then republishes it 
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class NthLaserScan : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Nth Laser Scan object
     * 
     */
    NthLaserScan() : Node("laser_scan_processor"){
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NthLaserScan::scanCallback, this, std::placeholders::_1));

        scan_nth_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_nth", 10); // Make a new publisher
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_nth_pub;


    /**
     * @brief a repitious method called to gather subscriber data and publish new data
     * 
     * @param scan the laser scan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
        int nthIncrement = 1;
        int totalNumOfScans = scan -> ranges.size();


        // Make a new message
        auto scan_nth = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        scan_nth->angle_min = scan->angle_min;
        scan_nth->angle_max = scan->angle_max;
        scan_nth->ranges = std::vector<float>();

        // scan_nth->ranges = std::vector<float>(scan->ranges.begin(), scan->ranges.end());

        
        for(int i = 0; i < totalNumOfScans; i += nthIncrement){
            scan_nth->ranges.push_back(scan->ranges[i]);
        }
        
        scan_nth->angle_increment = ((scan_nth->angle_max-scan_nth->angle_min)/scan_nth->ranges.size());
        RCLCPP_INFO(this->get_logger(), "Turned %d scans into %lu scans.\t Increment from %f to %f", totalNumOfScans, scan_nth->ranges.size(), scan->angle_increment, scan_nth->angle_increment);
        RCLCPP_INFO(this->get_logger(), "increment %d (angle max %f, min %f, nth max %f, min %f)", nthIncrement, scan-> angle_max, scan->angle_min, scan_nth-> angle_max, scan_nth->angle_min);
        
        scan_nth_pub->publish(*scan_nth);
    }

  
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NthLaserScan>());
    rclcpp::shutdown();
    return 0;
}
 