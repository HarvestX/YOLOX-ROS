#include "yolox_ros_cpp/using_service_v4l2camera.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace using_service_v4l2camera
{
    void using_service::yolox_callback()
    {
        RCLCPP_INFO(this->get_logger(), "yolox_callback");
        while (!client_yolox->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Client interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service...");
            break;
        }

        request = std::make_shared<yolo_msgs::srv::DetectObject::Request>();
        request->image = *image_msg;

        future_yolox = client_yolox->async_send_request(request, std::bind(&using_service::callback_response, this, _1));
    }

    void using_service::callback_response(rclcpp::Client<yolo_msgs::srv::DetectObject>::SharedFuture future)
    {
        auto response = future.get();
        std::vector<yolo_msgs::msg::BoundingBox> boundingboxes;

        for (auto &box : response->bounding_boxes)
        {
            yolo_msgs::msg::BoundingBox boundingbox;
            boundingbox.xmin = box.xmin;
            boundingbox.ymin = box.ymin;
            boundingbox.xmax = box.xmax;
            boundingbox.ymax = box.ymax;
            boundingbox.class_id = box.class_id;
            boundingbox.confidence = box.confidence;
            boundingboxes.push_back(boundingbox);
        }

        // print all
        for (auto &bbox : response->bounding_boxes)
        {
            RCLCPP_INFO(this->get_logger(), "xmin: %d, ymin: %d, xmax: %d, ymax: %d, class_id: %s, confidence: %f",
                        bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.class_id.c_str(), bbox.confidence);
        }

        // Publish bboxes
        yolo_msgs::msg::BoundingBoxes boundingboxes_msg;
        boundingboxes_msg.bounding_boxes = boundingboxes;
        pub_boundingboxes->publish(boundingboxes_msg);

        // Draw bboxes
        if (this->imshow_is_show)
        {
            draw_objects(frame, boundingboxes);
            cv::imshow("frame", frame);
            cv::waitKey(1);
        }
    }

    // Subscription
    void using_service::callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        frame = cv_ptr->image;
        image_msg = msg;
        yolox_callback();
    }

    using_service::using_service(const rclcpp::NodeOptions &options) : Node("using_service", options)
    {
        this->declare_parameter<std::string>("class_yaml", "");
        this->get_parameter("class_yaml", this->yaml_file_name_);

        this->declare_parameter<bool>("imshow_is_show", true);
        this->get_parameter("imshow_is_show", this->imshow_is_show);

        if (this->yaml_file_name_.size() > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Set parameter class_yaml file path: '%s'", this->yaml_file_name_.c_str());
            if (this->loadYaml(this->yaml_file_name_))
            {
                RCLCPP_INFO(this->get_logger(), "Load class_yaml file success");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Load class_yaml file failed");
                exit(1);
            }
        }
        

        client_yolox = this->create_client<yolo_msgs::srv::DetectObject>("detect_object");
        sub_image = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&using_service::callback_image, this, _1));
        pub_boundingboxes = this->create_publisher<yolo_msgs::msg::BoundingBoxes>("boundingboxes", 10);
    }

    void using_service::draw_objects(cv::Mat bgr, const std::vector<yolo_msgs::msg::BoundingBox> &boundingboxes)
    {
        for (size_t i = 0; i < boundingboxes.size(); i++)
        {
            int label_id = name2id(boundingboxes[i].class_id);

            cv::Scalar color = cv::Scalar(this->coco_data[label_id].rgb.b,
                                          this->coco_data[label_id].rgb.g,
                                          this->coco_data[label_id].rgb.r);
            float c_mean = cv::mean(color)[0];
            cv::Scalar txt_color;
            if (c_mean > 0.5)
            {
                txt_color = cv::Scalar(0, 0, 0);
            }
            else
            {
                txt_color = cv::Scalar(255, 255, 255);
            }

            cv::rectangle(bgr, cv::Point(boundingboxes[i].xmin, boundingboxes[i].ymin),
                          cv::Point(boundingboxes[i].xmax, boundingboxes[i].ymax), color*255, 2);

            char text[256];
            sprintf(text, "%s %.1f%%", this->coco_data[label_id].name.c_str(), boundingboxes[i].confidence * 100);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

            cv::Scalar txt_bk_color = color * 0.7 * 255;

            int x = boundingboxes[i].xmin;
            int y = boundingboxes[i].ymin;

            if (y > bgr.rows)
                y = bgr.rows;

            cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                          txt_bk_color, -1);

            cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
        }
    }

    int using_service::name2id(std::string name)
    {
        for (unsigned int i = 0; i < this->coco_data.size(); i++)
        {
            if (this->coco_data[i].name == name)
            {
                return i;
            }
        }
        return -1;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(using_service_v4l2camera::using_service)