#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp> 
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/ximgproc.hpp>

using std::placeholders::_1;

class ImageCapture : public rclcpp::Node
{
public:

    ImageCapture() : Node("Image_capture")
    {
        subscriber_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImageCapture::opencv_callback, this, _1));
        // control_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/autocar/cmd_vel", 10);
        //Initialize OpenCV window 
        cv::namedWindow("Image", cv::WINDOW_NORMAL); 
        cv::namedWindow("Processed_Image", cv::WINDOW_NORMAL);    
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;

    void opencv_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat image = cv_ptr->image;
            cv::imshow("Image",image);
            cv::waitKey(1);
            cv::Mat processed_frame = frame_processor(image);
            // cv::imshow("Processed_Image",processed_frame);
        }
        catch (const cv_bridge::Exception& e) {
            // Handle the exception (e.g., log an error message)
            RCLCPP_ERROR(get_logger(), "Error converting ROS image message to OpenCV image: %s", e.what());
        }
    }

    cv::Mat frame_processor(cv::Mat& image) {

        cv::Mat grayscale, blur, edges, region, hough, line_img;

        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayscale, blur, cv::Size(5, 5), 0);
        cv::Canny(blur, edges, 80, 110);
        region = region_selection(edges);
        std::vector<cv::Vec4i> lines = hough_transform(region);
        line_img = slope_lines(image,lines);
        cv::Mat dup = lane_lines(image, lines, cv::Scalar(255,255,0), 12);
        cv::imshow("Processed_Image",dup);
        return line_img;
    }

    cv::Mat region_selection(cv::Mat& image) {
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        std::vector<cv::Point> vertices = {
            cv::Point(image.cols * 0.1, image.rows * 0.95),
            cv::Point(image.cols * 0.4, image.rows * 0.6),
            cv::Point(image.cols * 0.6, image.rows * 0.6),
            cv::Point(image.cols * 0.9, image.rows * 0.95)
        };
        cv::fillConvexPoly(mask, vertices, cv::Scalar(255));
        cv::Mat masked_image;
        cv::bitwise_and(image, mask, masked_image);
        return masked_image;
    }

    std::vector<cv::Vec4i> hough_transform(cv::Mat& image) {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(image, lines, 1, CV_PI / 180, 20, 20, 180);
        return lines;
    }

    cv::Mat slope_lines(cv::Mat& image, std::vector<cv::Vec4i> lines) {
        cv::Mat img = image.clone();

        std::vector<cv::Point> poly_vertices;
        int order[] = {0, 1, 3, 2};

        std::vector<std::pair<double, double>> left_lines;
        std::vector<std::pair<double, double>> right_lines;

        for (size_t i = 0; i < lines.size(); i++) {
            int x1 = lines[i][0], y1 = lines[i][1], x2 = lines[i][2], y2 = lines[i][3];

            if (x1 == x2) 
            continue; // Vertical Lines
            else 
            {
                double m = static_cast<double>(y2 - y1) / (x2 - x1);
                double c = y1 - m * x1;

                if (m < 0) 
                left_lines.push_back(std::make_pair(m, c));
                else 
                right_lines.push_back(std::make_pair(m, c));

            }
        }

        double left_slope = 0, left_intercept = 0, right_slope = 0, right_intercept = 0;

        for (auto& line : left_lines) {
            left_slope += line.first;
            left_intercept += line.second;
        }
        left_slope /= left_lines.size();
        left_intercept /= left_lines.size();
        std::pair<double, double> left_line = std::make_pair(left_slope, left_intercept);

        for (auto& line : right_lines) {
            right_slope += line.first;
            right_intercept += line.second;
        }
        right_slope /= right_lines.size();
        right_intercept /= right_lines.size();
        std::pair<double, double> right_line = std::make_pair(right_slope, right_intercept);

        for (auto& line : {left_line, right_line}) {
            int rows = image.rows;

            int y1 = rows;
            int y2 = rows * 0.6;

            int x1 = static_cast<int>((y1 - line.second) / line.first);
            int x2 = static_cast<int>((y2 - line.second) / line.first);

            poly_vertices.push_back(cv::Point(x1, y1));
            poly_vertices.push_back(cv::Point(x2, y2));

            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
        }

        std::vector<cv::Point> poly_vertices_ordered;
        for (int i = 0; i < 4; ++i) {
            poly_vertices_ordered.push_back(poly_vertices[order[i]]);
        }

        std::vector<std::vector<cv::Point>> pts{poly_vertices_ordered};
        cv::fillPoly(img, pts, cv::Scalar(0, 255, 0));
        cv::addWeighted(image, 0.7, img, 0.4, 0.0, image);
        cv::addWeighted(image, 0.8, img, 1, 0.0, image);
        // cv::imshow("Processed_Image",img);
        return image;
    }

    cv::Mat lane_lines(cv::Mat& image, std::vector<cv::Vec4i>& lines, cv::Scalar color, int thickness) {
        // cv::Vec4i left_line, right_line;
        cv::Mat dup = image;
        for (cv::Vec4i line : lines) {

            cv::line(dup, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, thickness, cv::LINE_AA);

        }
        return dup;
    }
        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCapture>());
    rclcpp::shutdown();
    return 0;
}

