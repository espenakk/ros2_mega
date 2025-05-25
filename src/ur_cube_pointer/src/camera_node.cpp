#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "custom_interfaces/msg/detected_cube.hpp"
#include "custom_interfaces/msg/detected_cubes.hpp"
#include "std_msgs/msg/header.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("image_topic", "/image_raw");
        this->declare_parameter<double>("min_contour_area", 500.0);

        // Red
        this->declare_parameter<int>("h_min_red1", 0);
        this->declare_parameter<int>("s_min_red1", 120);
        this->declare_parameter<int>("v_min_red1", 70);
        this->declare_parameter<int>("h_max_red1", 10);
        this->declare_parameter<int>("s_max_red1", 255);
        this->declare_parameter<int>("v_max_red1", 255);
        this->declare_parameter<int>("h_min_red2", 170);
        this->declare_parameter<int>("s_min_red2", 120);
        this->declare_parameter<int>("v_min_red2", 70);
        this->declare_parameter<int>("h_max_red2", 180);
        this->declare_parameter<int>("s_max_red2", 255);
        this->declare_parameter<int>("v_max_red2", 255);
        // Yellow
        this->declare_parameter<int>("h_min_yellow", 20);
        this->declare_parameter<int>("s_min_yellow", 100);
        this->declare_parameter<int>("v_min_yellow", 100);
        this->declare_parameter<int>("h_max_yellow", 30);
        this->declare_parameter<int>("s_max_yellow", 255);
        this->declare_parameter<int>("v_max_yellow", 255);
        // Blue
        this->declare_parameter<int>("h_min_blue", 90);
        this->declare_parameter<int>("s_min_blue", 80);
        this->declare_parameter<int>("v_min_blue", 80);
        this->declare_parameter<int>("h_max_blue", 130);
        this->declare_parameter<int>("s_max_blue", 255);
        this->declare_parameter<int>("v_max_blue", 255);
        // Green - New Parameters
        this->declare_parameter<int>("h_min_green", 35);
        this->declare_parameter<int>("s_min_green", 80);
        this->declare_parameter<int>("v_min_green", 70);
        this->declare_parameter<int>("h_max_green", 85);
        this->declare_parameter<int>("s_max_green", 255);
        this->declare_parameter<int>("v_max_green", 255);


        // Get parameters
        image_topic_ = this->get_parameter("image_topic").as_string();
        min_contour_area_ = this->get_parameter("min_contour_area").as_double();

        // Red1
        h_min_red1_ = this->get_parameter("h_min_red1").as_int();
        s_min_red1_ = this->get_parameter("s_min_red1").as_int();
        v_min_red1_ = this->get_parameter("v_min_red1").as_int();
        h_max_red1_ = this->get_parameter("h_max_red1").as_int();
        s_max_red1_ = this->get_parameter("s_max_red1").as_int();
        v_max_red1_ = this->get_parameter("v_max_red1").as_int();

        // Red2
        h_min_red2_ = this->get_parameter("h_min_red2").as_int();
        s_min_red2_ = this->get_parameter("s_min_red2").as_int();
        v_min_red2_ = this->get_parameter("v_min_red2").as_int();
        h_max_red2_ = this->get_parameter("h_max_red2").as_int();
        s_max_red2_ = this->get_parameter("s_max_red2").as_int();
        v_max_red2_ = this->get_parameter("v_max_red2").as_int();

        // Yellow
        h_min_yellow_ = this->get_parameter("h_min_yellow").as_int();
        s_min_yellow_ = this->get_parameter("s_min_yellow").as_int();
        v_min_yellow_ = this->get_parameter("v_min_yellow").as_int();
        h_max_yellow_ = this->get_parameter("h_max_yellow").as_int();
        s_max_yellow_ = this->get_parameter("s_max_yellow").as_int();
        v_max_yellow_ = this->get_parameter("v_max_yellow").as_int();

        // Blue
        h_min_blue_ = this->get_parameter("h_min_blue").as_int();
        s_min_blue_ = this->get_parameter("s_min_blue").as_int();
        v_min_blue_ = this->get_parameter("v_min_blue").as_int();
        h_max_blue_ = this->get_parameter("h_max_blue").as_int();
        s_max_blue_ = this->get_parameter("s_max_blue").as_int();
        v_max_blue_ = this->get_parameter("v_max_blue").as_int();

        // Green
        h_min_green_ = this->get_parameter("h_min_green").as_int();
        s_min_green_ = this->get_parameter("s_min_green").as_int();
        v_min_green_ = this->get_parameter("v_min_green").as_int();
        h_max_green_ = this->get_parameter("h_max_green").as_int();
        s_max_green_ = this->get_parameter("s_max_green").as_int();
        v_max_green_ = this->get_parameter("v_max_green").as_int();

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

        cubes_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCubes>("/detected_cubes", 10);

        RCLCPP_INFO(this->get_logger(), "Camera node started. Subscribing to %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing detected cubes to /detected_cubes");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ... (rest of the image_callback function preamble: cv_bridge, frame check)
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image; 
            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty frame on topic %s", image_topic_.c_str());
                return;
            }

            custom_interfaces::msg::DetectedCubes detected_cubes_msg;
            detected_cubes_msg.header = msg->header;

            detect_color(frame, "red", cv::Scalar(h_min_red1_, s_min_red1_, v_min_red1_), cv::Scalar(h_max_red1_, s_max_red1_, v_max_red1_),
                         cv::Scalar(h_min_red2_, s_min_red2_, v_min_red2_), cv::Scalar(h_max_red2_, s_max_red2_, v_max_red2_),
                         detected_cubes_msg);

            detect_color(frame, "yellow", cv::Scalar(h_min_yellow_, s_min_yellow_, v_min_yellow_), cv::Scalar(h_max_yellow_, s_max_yellow_, v_max_yellow_),
                         cv::Scalar(0,0,0), cv::Scalar(0,0,0), 
                         detected_cubes_msg);

            detect_color(frame, "blue", cv::Scalar(h_min_blue_, s_min_blue_, v_min_blue_), cv::Scalar(h_max_blue_, s_max_blue_, v_max_blue_),
                         cv::Scalar(0,0,0), cv::Scalar(0,0,0), 
                         detected_cubes_msg);
            
            // Add detection for Green
            detect_color(frame, "green", cv::Scalar(h_min_green_, s_min_green_, v_min_green_), cv::Scalar(h_max_green_, s_max_green_, v_max_green_),
                         cv::Scalar(0,0,0), cv::Scalar(0,0,0), // Green typically doesn't wrap around Hue like red
                         detected_cubes_msg);


            if (!detected_cubes_msg.cubes.empty()) {
                RCLCPP_INFO(this->get_logger(), "Detected %zu cubes.", detected_cubes_msg.cubes.size());
                for(const auto& cube : detected_cubes_msg.cubes) {
                    RCLCPP_INFO(this->get_logger(), "  - %s cube at (%.0f, %.0f)", cube.color.c_str(), cube.position.x, cube.position.y);
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "No cubes detected in this frame.");
            }
            cubes_publisher_->publish(detected_cubes_msg);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Standard exception in image_callback: %s", e.what());
        }
    }

    // detect_color function remains the same
    void detect_color(const cv::Mat& frame_to_process, const std::string& color_name,
                      cv::Scalar hsv_min1, cv::Scalar hsv_max1,
                      cv::Scalar hsv_min2, cv::Scalar hsv_max2, // Only used if second range is needed (e.g. red)
                      custom_interfaces::msg::DetectedCubes& detected_cubes_msg)
    {
        // ... (implementation of detect_color is unchanged) ...
        cv::Mat hsv_frame, mask1, mask2, mask;
        cv::cvtColor(frame_to_process, hsv_frame, cv::COLOR_BGR2HSV);

        cv::inRange(hsv_frame, hsv_min1, hsv_max1, mask1);
        // Check if hsv_min2 and hsv_max2 are non-zero (or some other indicator that they should be used)
        // For red, hsv_min2 usually starts at a high H value (e.g., 170). For other colors, we pass (0,0,0)
        bool use_second_range = (hsv_min2[0] > 0 || hsv_min2[1] > 0 || hsv_min2[2] > 0 ||
                                 hsv_max2[0] > 0 || hsv_max2[1] > 0 || hsv_max2[2] > 0);


        if (use_second_range && (color_name == "red")) { // Be specific if only red uses two ranges
             cv::inRange(hsv_frame, hsv_min2, hsv_max2, mask2);
             mask = mask1 | mask2;
        } else {
            mask = mask1;
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > min_contour_area_)
            {
                cv::Rect bounding_rect = cv::boundingRect(contour);

                geometry_msgs::msg::Point center;
                center.x = bounding_rect.x + bounding_rect.width / 2.0;
                center.y = bounding_rect.y + bounding_rect.height / 2.0;
                center.z = 0; 

                custom_interfaces::msg::DetectedCube detected_cube;
                detected_cube.color = color_name;
                detected_cube.position = center;
                detected_cubes_msg.cubes.push_back(detected_cube);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::DetectedCubes>::SharedPtr cubes_publisher_;
    std::string image_topic_;
    double min_contour_area_;

    // HSV Thresholds
    int h_min_red1_, s_min_red1_, v_min_red1_, h_max_red1_, s_max_red1_, v_max_red1_;
    int h_min_red2_, s_min_red2_, v_min_red2_, h_max_red2_, s_max_red2_, v_max_red2_;
    int h_min_yellow_, s_min_yellow_, v_min_yellow_, h_max_yellow_, s_max_yellow_, v_max_yellow_;
    int h_min_blue_, s_min_blue_, v_min_blue_, h_max_blue_, s_max_blue_, v_max_blue_;
    // Green - New Member Variables
    int h_min_green_, s_min_green_, v_min_green_, h_max_green_, s_max_green_, v_max_green_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}