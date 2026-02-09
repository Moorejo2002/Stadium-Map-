#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LaneVisualizer : public rclcpp::Node
{
public:
    LaneVisualizer() : Node("lane_visualizer")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&LaneVisualizer::image_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane/image_processed", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lane/marker_array", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "LaneVisualizer with Marker Started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        frame.convertTo(frame, -1, 1.0, -100);

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0,0,150), cv::Scalar(180,50,255), mask);

        // Marker 초기화
        visualization_msgs::msg::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = msg->header.stamp;
        points.ns = "lane_dots";
        points.id = 0;
        points.type = visualization_msgs::msg::Marker::POINTS;
        points.action = visualization_msgs::msg::Marker::ADD;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.r = 1.0;
        points.color.g = 0.0;
        points.color.b = 0.0;
        points.color.a = 1.0;

        int step = 30; 
        for(int y=0; y<mask.rows; y+=step) {
            const uchar* row_ptr = mask.ptr<uchar>(y);
            bool in_lane = false;
            int start_x = 0;
            for(int x=0; x<mask.cols; x++) {
                if(row_ptr[x] > 0) {
                    if(!in_lane) { in_lane = true; start_x = x; }
                } else {
                    if(in_lane) {
                        in_lane = false;
                        int end_x = x-1;
                        int cx = (start_x+end_x)/2;
                        int cy = y;
                        cv::circle(frame, cv::Point(cx,cy), 5, cv::Scalar(0,0,255), -1);

                        // 카메라 좌표 -> map 좌표
                        geometry_msgs::msg::PointStamped p_cam;
                        p_cam.header.frame_id = "camera_link";
                        p_cam.header.stamp = msg->header.stamp;
                        double u = cx - frame.cols/2.0;
                        double v = cy - frame.rows/2.0;
                        double z = 0.5 + (mask.rows-cy)*0.02;
                        p_cam.point.x = u*0.01*z;
                        p_cam.point.y = -v*0.01*z;
                        p_cam.point.z = z;

                        try {
                            geometry_msgs::msg::PointStamped p_map;
                            p_map = tf_buffer_->transform(p_cam, "map", tf2::durationFromSec(0.1));
                            points.points.push_back(p_map.point);
                        } catch(tf2::TransformException &ex) {
                            RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
                        }
                    }
                }
            }
        }

        // 퍼블리시
        image_pub_->publish(*cv_ptr->toImageMsg());
        marker_pub_->publish(points);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneVisualizer>());
    rclcpp::shutdown();
    return 0;
}

