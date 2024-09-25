#include <scan_to_image.h>


ScanToImageNode::ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0) {
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
}

void ScanToImageNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Scancallback started.");
   // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
    cv::Mat img = laserScanToMat(msg);
    RCLCPP_DEBUG(this->get_logger(), "first_image_captured_: "+first_image_captured_);
    RCLCPP_DEBUG(this->get_logger(), "second_image_captured_: "+second_image_captured_);


    if (!first_image_captured_) {
        first_image_ = img.clone();
        first_image_captured_ = true;
        // Display the first image
        cv::imshow("First Image", first_image_);
        cv::waitKey(1);  // Add this to process GUI events and update the window
        // Rotate the robot by publishing to cmd_vel
        // rotateRobot();
    } else if (!second_image_captured_) {
        second_image_ = img.clone();
        second_image_captured_ = true;
        // Display the second image
        cv::imshow("Second Image", second_image_);
        cv::waitKey(1);  // Add this to process GUI events and update the window
        // Calculate the change in yaw using cv::transform
    } else {
        first_image_ = second_image_.clone();
        second_image_ = img.clone();
        // Display the new second image
        cv::imshow("Second Image", second_image_);
        cv::waitKey(1);  // Add this to process GUI events and update the window

        calculateYawChange();
        relative_orientaion_ = relative_orientaion_ + angle_difference_;
        RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
    }
}

/*
cv::Mat ScanToImageNode::laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    RCLCPP_DEBUG(this->get_logger(), "Laser scan to cv::mat.");

    // Parameters
    int img_size = 500;
    float max_range = scan->range_max;
    cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

    for (size_t i = 0; i < scan->ranges.size(); i++) {
        float range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max) {
            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
            int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
            if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                image.at<uchar>(y, x) = 255;
            }
        }
    }
    return image;
}
*/

cv::Mat ScanToImageNode::laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    
    // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

    // Functionality:
    //      Create Image: Initializes a blank image of size 500x500 pixels.
    int dimension = 500;
    cv::Mat laserScanImage = cv::Mat(dimension, dimension, CV_8UC1); // Greyscale img

    // Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.

    // std::vector<std::pair<int, int>> xy_coords;

    // // Loop through each laser point and calcualte (x, y)
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        // Calculate the angle for this measurement
        float angle = scan->angle_min + i * scan->angle_increment;
        float range = scan->ranges[i];

        float x = range * std::cos(angle);
        float y = range * std::sin(angle);

        // xy_coords.push_back({x, y});
        laserScanImage.at<uchar>(y, x) = 255; // white for laser scan point there
    }

    return laserScanImage;

}

void ScanToImageNode::calculateYawChange() {
    // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

    // Functionality:

    //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
                // std::vector<cv::Point2f> srcPoints, dstPoints;
                // detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
    //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
    //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.


        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }

}

void ScanToImageNode::detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                            std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
    RCLCPP_DEBUG(this->get_logger(), "DetectAndMatchFeatures started.");
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // Sort matches based on distance (lower distance means better match)
    std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
        return a.distance < b.distance;
    });

    // Determine the number of top matches to keep (30% of total matches)
    size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

    // Keep only the best matches (top 30%)
    std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

    for (const auto& match : matches) {
        srcPoints.push_back(keypoints1[match.queryIdx].pt);
        dstPoints.push_back(keypoints2[match.trainIdx].pt);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}