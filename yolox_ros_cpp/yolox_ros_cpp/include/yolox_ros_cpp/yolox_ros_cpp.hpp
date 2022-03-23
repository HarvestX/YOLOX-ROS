#ifndef _YOLOX_ROS_CPP_YOLOX_ROS_CPP_HPP
#define _YOLOX_ROS_CPP_YOLOX_ROS_CPP_HPP
#include <math.h>
#include <chrono>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <yolo_msgs/msg/bounding_box.hpp>
#include <yolo_msgs/msg/bounding_boxes.hpp>
#include <yolo_msgs/srv/detect_object.hpp>

#include <yolox_cpp/yolox.hpp>
#include <yolox_cpp/utils.hpp>

#include "yolox_ros_cpp/bboxes_from_yaml.hpp"
namespace yolox_ros_cpp
{

    class YoloXNode : public rclcpp::Node, public bboxes_from_yaml
    {
    public:
        YoloXNode(const rclcpp::NodeOptions &options);
        YoloXNode(const std::string &node_name, const rclcpp::NodeOptions &options);

    private:
        void initializeParameter();
        std::unique_ptr<yolox_cpp::AbsYoloX> yolox_;
        std::string model_path_;
        std::string model_type_;
        std::string device_;
        float conf_th_;
        float nms_th_;
        int image_width_;
        int image_height_;

        std::string src_image_topic_name_;
        std::string publish_image_topic_name_;
        std::string publish_boundingbox_topic_name_;

        image_transport::Subscriber sub_image_;
        void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr);
        void colorImageSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<yolo_msgs::srv::DetectObject::Request> req,
                                   std::shared_ptr<yolo_msgs::srv::DetectObject::Response> res);

        rclcpp::Publisher<yolo_msgs::msg::BoundingBoxes>::SharedPtr pub_bboxes_;
        image_transport::Publisher pub_image_;
        rclcpp::Service<yolo_msgs::srv::DetectObject>::SharedPtr srv_detect_object_;

        yolo_msgs::msg::BoundingBoxes objects_to_bboxes(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::msg::Header header);
        std::vector<yolo_msgs::msg::BoundingBox> objects_to_bbox_vec(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::msg::Header header);

        std::string getModelPath(const std::string &model_path);

        std::string WINDOW_NAME_ = "YOLOX";
        bool imshow_ = true;

        // bboxes_from_yaml bboxes;
        void draw_objects(cv::Mat bgr, const std::vector<yolox_cpp::Object> &objects);
        std::string yaml_file_name_;
    };
}
#endif