#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath> 


class ScanToImageNode : public rclcpp::Node {
    public:
        ScanToImageNode();

    private:
        cv::Mat first_image_, second_image_;
        bool first_image_captured_ = false;
        bool second_image_captured_ = false;

        double angle_difference_;
        double relative_orientaion_ = 0.0;


        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;


        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

        void calculateYawChange();
        void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints);
};