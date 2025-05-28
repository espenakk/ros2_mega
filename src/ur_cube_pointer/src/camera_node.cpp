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

        // Red parameters
        this->declare_parameter<int>("h_min_red1", 0);
        this->declare_parameter<int>("s_min_red1", 120);
        this->declare_parameter<int>("v_min_red1", 70);
        this->declare_parameter<int>("h_max_red1", 10);
        this->declare_parameter<int>("s_max_red1", 255);
        this->declare_parameter<int>("v_max_red1", 255);

        // Yellow parameters
        this->declare_parameter<int>("h_min_yellow", 20);
        this->declare_parameter<int>("s_min_yellow", 100);
        this->declare_parameter<int>("v_min_yellow", 100);
        this->declare_parameter<int>("h_max_yellow", 30);
        this->declare_parameter<int>("s_max_yellow", 255);
        this->declare_parameter<int>("v_max_yellow", 255);

        // Blue parameters
        this->declare_parameter<int>("h_min_blue", 90);
        this->declare_parameter<int>("s_min_blue", 80);
        this->declare_parameter<int>("v_min_blue", 80);
        this->declare_parameter<int>("h_max_blue", 130);
        this->declare_parameter<int>("s_max_blue", 255);
        this->declare_parameter<int>("v_max_blue", 255);

        // Get parameter values
        image_topic_ = this->get_parameter("image_topic").as_string();
        min_contour_area_ = this->get_parameter("min_contour_area").as_double();

        // Red1 parameters
        h_min_red1_ = this->get_parameter("h_min_red1").as_int();
        s_min_red1_ = this->get_parameter("s_min_red1").as_int();
        v_min_red1_ = this->get_parameter("v_min_red1").as_int();
        h_max_red1_ = this->get_parameter("h_max_red1").as_int();
        s_max_red1_ = this->get_parameter("s_max_red1").as_int();
        v_max_red1_ = this->get_parameter("v_max_red1").as_int();

        // Yellow parameters
        h_min_yellow_ = this->get_parameter("h_min_yellow").as_int();
        s_min_yellow_ = this->get_parameter("s_min_yellow").as_int();
        v_min_yellow_ = this->get_parameter("v_min_yellow").as_int();
        h_max_yellow_ = this->get_parameter("h_max_yellow").as_int();
        s_max_yellow_ = this->get_parameter("s_max_yellow").as_int();
        v_max_yellow_ = this->get_parameter("v_max_yellow").as_int();

        // Blue parameters
        h_min_blue_ = this->get_parameter("h_min_blue").as_int();
        s_min_blue_ = this->get_parameter("s_min_blue").as_int();
        v_min_blue_ = this->get_parameter("v_min_blue").as_int();
        h_max_blue_ = this->get_parameter("h_max_blue").as_int();
        s_max_blue_ = this->get_parameter("s_max_blue").as_int();
        v_max_blue_ = this->get_parameter("v_max_blue").as_int();

        // Homography setup
        std::vector<cv::Point2f> image_points = {
            cv::Point2f(788, 555),   // Top Left
            cv::Point2f(1043, 695),  // Top Right
            cv::Point2f(1140, 516),  // Bottom Right
            cv::Point2f(884, 393)    // Bottom Left
        };

        std::vector<cv::Point2f> world_points = {
            cv::Point2f(0.0f, 0.0f),    // Top Left
            cv::Point2f(0.45f, 0.0f),   // Top Right
            cv::Point2f(0.45f, 0.30f),  // Bottom Right
            cv::Point2f(0.0f, 0.30f)    // Bottom Left
        };

        H_ = cv::findHomography(image_points, world_points);

        // Initialize subscriptions and publishers
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

        cubes_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCubes>("/detected_cubes", 10);
        red_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCube>("/red_cube_coordinate", 10);
        yellow_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCube>("/yellow_cube_coordinate", 10);
        blue_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCube>("/blue_cube_coordinate", 10);
        green_publisher_ = this->create_publisher<custom_interfaces::msg::DetectedCube>("/green_cube_coordinate", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized with:");
        RCLCPP_INFO(this->get_logger(), " - Image topic: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), " - Min contour area: %.2f", min_contour_area_);
    }

private:
    geometry_msgs::msg::Point transform_to_robot_coords(const geometry_msgs::msg::Point& pixel_point)
    {
        std::vector<cv::Point2f> src = {cv::Point2f(pixel_point.x, pixel_point.y)};
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, H_);

        geometry_msgs::msg::Point world_point;
        world_point.x = dst[0].x;
        world_point.y = dst[0].y;
        world_point.z = 0.0;
        return world_point;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image frame");
                return;
            }

            // Preprocess image
            cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);

            custom_interfaces::msg::DetectedCubes detected_cubes_msg;
            detected_cubes_msg.header = msg->header;

            // Detect colors
            detect_color(frame, "red",
                        cv::Scalar(h_min_red1_, s_min_red1_, v_min_red1_),
                        cv::Scalar(h_max_red1_, s_max_red1_, v_max_red1_),
                        detected_cubes_msg);

            detect_color(frame, "yellow",
                        cv::Scalar(h_min_yellow_, s_min_yellow_, v_min_yellow_),
                        cv::Scalar(h_max_yellow_, s_max_yellow_, v_max_yellow_),
                        cv::Scalar(0,0,0), cv::Scalar(0,0,0),
                        detected_cubes_msg);

            detect_color(frame, "blue",
                        cv::Scalar(h_min_blue_, s_min_blue_, v_min_blue_),
                        cv::Scalar(h_max_blue_, s_max_blue_, v_max_blue_),
                        cv::Scalar(0,0,0), cv::Scalar(0,0,0),
                        detected_cubes_msg);

            // Publish aggregated cubes
            if (!detected_cubes_msg.cubes.empty()) {
                cubes_publisher_->publish(detected_cubes_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published %zu cubes", detected_cubes_msg.cubes.size());
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
        }
    }

    void detect_color(const cv::Mat& frame, const std::string& color_name,
                     cv::Scalar hsv_min1, cv::Scalar hsv_max1,
                     cv::Scalar hsv_min2, cv::Scalar hsv_max2,
                     custom_interfaces::msg::DetectedCubes& output_msg)
    {
        cv::Mat hsv, mask1, mask2, combined_mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Create primary mask
        cv::inRange(hsv, hsv_min1, hsv_max1, mask1);

        // Handle dual-range colors (like red)
        bool use_secondary = color_name == "red" &&
                            (hsv_min2[0] > 0 || hsv_min2[1] > 0 || hsv_min2[2] > 0);

        if (use_secondary) {
            cv::inRange(hsv, hsv_min2, hsv_max2, mask2);
            combined_mask = mask1 | mask2;
        } else {
            combined_mask = mask1;
        }

        // Noise reduction
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN,
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE,
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > min_contour_area_) {
                cv::Rect bounds = cv::boundingRect(contour);

                // Calculate center point
                geometry_msgs::msg::Point pixel_center;
                pixel_center.x = bounds.x + bounds.width/2.0;
                pixel_center.y = bounds.y + bounds.height/2.0;
                pixel_center.z = 0;

                // Create detection message
                custom_interfaces::msg::DetectedCube cube;
                cube.color = color_name;
                cube.position = transform_to_robot_coords(pixel_center);

                // Add to aggregated message
                output_msg.cubes.push_back(cube);

                // Publish to color-specific topic
                if (color_name == "red") {
                    red_publisher_->publish(cube);
                } else if (color_name == "yellow") {
                    yellow_publisher_->publish(cube);
                } else if (color_name == "blue") {
                    blue_publisher_->publish(cube);
                } 
            }
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::DetectedCubes>::SharedPtr cubes_publisher_;
    rclcpp::Publisher<custom_interfaces::msg::DetectedCube>::SharedPtr red_publisher_;
    rclcpp::Publisher<custom_interfaces::msg::DetectedCube>::SharedPtr yellow_publisher_;
    rclcpp::Publisher<custom_interfaces::msg::DetectedCube>::SharedPtr blue_publisher_;

    std::string image_topic_;
    double min_contour_area_;
    cv::Mat H_;

    // HSV parameters
    int h_min_red1_, s_min_red1_, v_min_red1_, h_max_red1_, s_max_red1_, v_max_red1_;
    int h_min_yellow_, s_min_yellow_, v_min_yellow_, h_max_yellow_, s_max_yellow_, v_max_yellow_;
    int h_min_blue_, s_min_blue_, v_min_blue_, h_max_blue_, s_max_blue_, v_max_blue_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
