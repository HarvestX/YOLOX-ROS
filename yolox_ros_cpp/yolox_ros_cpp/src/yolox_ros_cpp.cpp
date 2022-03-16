#include "yolox_ros_cpp/yolox_ros_cpp.hpp"

namespace yolox_ros_cpp{

    YoloXNode::YoloXNode(const rclcpp::NodeOptions& options)
    : YoloXNode::YoloXNode("", options)
    {}

    YoloXNode::YoloXNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("yolox_ros_cpp", node_name, options){
        
        RCLCPP_INFO(this->get_logger(), "initialize");
        this->initializeParameter();

        if(this->imshow_){
            char window_name[50];
            sprintf(window_name, "%s %s %s", this->WINDOW_NAME_.c_str(), "_", this->get_name());
            this->WINDOW_NAME_ = window_name;            

            cv::namedWindow(this->WINDOW_NAME_, cv::WINDOW_AUTOSIZE);
        }
        
        if(this->model_type_ == "tensorrt"){
            #ifdef ENABLE_TENSORRT
                RCLCPP_INFO(this->get_logger(), "Model Type is TensorRT");
                this->yolox_ = std::make_unique<yolox_cpp::YoloXTensorRT>(this->model_path_, std::stoi(this->device_),
                                                                          this->nms_th_, this->conf_th_, 
                                                                          this->image_width_, this->image_height_);
            #else
                RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with TensorRT");
                rclcpp::shutdown();
            #endif
        }else if(this->model_type_ == "openvino"){
            #ifdef ENABLE_OPENVINO
                RCLCPP_INFO(this->get_logger(), "Model Type is OpenVINO");
                this->yolox_ = std::make_unique<yolox_cpp::YoloXOpenVINO>(this->model_path_, this->device_,
                                                                          this->nms_th_, this->conf_th_,
                                                                          this->image_width_, this->image_height_);
            #else
                RCLCPP_ERROR(this->get_logger(), "yolox_cpp is not built with OpenVINO");
                rclcpp::shutdown();
            #endif
        }

        this->sub_image_ = image_transport::create_subscription(
            this, this->src_image_topic_name_, 
            std::bind(&YoloXNode::colorImageCallback, this, std::placeholders::_1), 
            "raw");
        this->pub_bboxes_ = this->create_publisher<yolo_msgs::msg::BoundingBoxes>(
            this->publish_boundingbox_topic_name_,
            10
        );
        this->pub_image_ = image_transport::create_publisher(this, this->publish_image_topic_name_);

    }

    void YoloXNode::initializeParameter(){
        this->declare_parameter<bool>("imshow_isshow", true);
        this->declare_parameter<std::string>("model_path", "/root/ros2_ws/src/YOLOX-ROS/weights/tensorrt/YOLOX_outputs/nano/model_trt.engine");
        this->declare_parameter<float>("conf", 0.3f);
        this->declare_parameter<float>("nms", 0.45f);
        this->declare_parameter<std::string>("device", "0");
        this->declare_parameter<int>("image_size/width", 416);
        this->declare_parameter<int>("image_size/height", 416);
        this->declare_parameter<std::string>("model_type", "tensorrt");
        this->declare_parameter<std::string>("src_image_topic_name", "image_raw");
        this->declare_parameter<std::string>("publish_image_topic_name", "yolox/image_raw");        
        this->declare_parameter<std::string>("publish_boundingbox_topic_name", "yolox/bounding_boxes");        

        this->declare_parameter<std::string>("class_yaml", "");

        this->get_parameter("imshow_isshow", this->imshow_);
        this->get_parameter("model_path", this->model_path_);
        this->get_parameter("conf", this->conf_th_);
        this->get_parameter("nms", this->nms_th_);
        this->get_parameter("device", this->device_);
        this->get_parameter("image_size/width", this->image_width_);
        this->get_parameter("image_size/height", this->image_height_);
        this->get_parameter("model_type", this->model_type_);
        this->get_parameter("src_image_topic_name", this->src_image_topic_name_);
        this->get_parameter("publish_image_topic_name", this->publish_image_topic_name_);
        this->get_parameter("publish_boundingbox_topic_name", this->publish_boundingbox_topic_name_);

        this->get_parameter("class_yaml", this->yaml_file_name_);

        RCLCPP_INFO(this->get_logger(), "Set parameter imshow_isshow: %i", this->imshow_);
        RCLCPP_INFO(this->get_logger(), "Set parameter model_path: '%s'", this->model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Set parameter conf: %f", this->conf_th_);
        RCLCPP_INFO(this->get_logger(), "Set parameter nms: %f", this->nms_th_);
        RCLCPP_INFO(this->get_logger(), "Set parameter device: %s", this->device_.c_str());
        RCLCPP_INFO(this->get_logger(), "Set parameter image_size/width: %i", this->image_width_);
        RCLCPP_INFO(this->get_logger(), "Set parameter image_size/height: %i", this->image_height_);
        RCLCPP_INFO(this->get_logger(), "Set parameter model_type: '%s'", this->model_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Set parameter src_image_topic_name: '%s'", this->src_image_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Set parameter publish_image_topic_name: '%s'", this->publish_image_topic_name_.c_str());

        RCLCPP_INFO(this->get_logger(), "Set parameter publish_boundingbox_topic_name: '%s'", this->publish_boundingbox_topic_name_.c_str());

        if (this->yaml_file_name_.size() > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Set parameter class_yaml file path: '%s'", this->yaml_file_name_.c_str());
            if (this->load_yaml(this->yaml_file_name_))
            {
                RCLCPP_INFO(this->get_logger(), "Load class_yaml file success");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Load class_yaml file failed");
            }
            
        }
    }
    void YoloXNode::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& ptr){
        auto img = cv_bridge::toCvCopy(ptr, "bgr8");
        cv::Mat frame = img->image;

        // fps
        auto now = std::chrono::system_clock::now();
        auto objects = this->yolox_->inference(frame);

        if (this->yaml_file_name_.size() > 0)
        {
            this->draw_objects(frame, objects);
        }
        else
        {
            yolox_cpp::utils::draw_objects(frame, objects);
        }

        // yolox_cpp::utils::draw_objects(frame, objects);
        if(this->imshow_){
            cv::imshow(this->WINDOW_NAME_, frame);
            auto key = cv::waitKey(1);
            if(key == 27){
                rclcpp::shutdown();
            }
        }

        auto boxes = objects_to_bboxes(frame, objects, img->header);
        this->pub_bboxes_->publish(boxes);

        sensor_msgs::msg::Image::SharedPtr pub_img;
        pub_img = cv_bridge::CvImage(img->header, "bgr8", frame).toImageMsg();
        this->pub_image_.publish(pub_img);

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        RCLCPP_INFO(this->get_logger(), "fps: %f", 1000.0f / elapsed.count());
    }
    yolo_msgs::msg::BoundingBoxes YoloXNode::objects_to_bboxes(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::msg::Header header){
        yolo_msgs::msg::BoundingBoxes boxes;
        // boxes.header = header;
        for(auto obj: objects){
            yolo_msgs::msg::BoundingBox box;
            // box.probability = obj.prob;
            box.confidence = obj.prob;
            // box.class_id = yolox_cpp::COCO_CLASSES[obj.label];
            box.class_id = this->coco_data[obj.label].name;
            box.xmin = obj.rect.x;
            box.ymin = obj.rect.y;
            box.xmax = (obj.rect.x + obj.rect.width);
            box.ymax = (obj.rect.y + obj.rect.height);
            // box.img_width = frame.cols;
            // box.img_height = frame.rows;
            // tracking id
            // box.id = 0;
            // depth
            // box.center_dist = 0;
            boxes.bounding_boxes.emplace_back(box);
        }
        return boxes;
    }

    void YoloXNode::draw_objects(cv::Mat bgr, const std::vector<yolox_cpp::Object> &objects)
    {

        // cv::Mat image = bgr.clone();

        for (size_t i = 0; i < objects.size(); i++)
        {
            const yolox_cpp::Object &obj = objects[i];

            // fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
            //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

            cv::Scalar color = cv::Scalar(this->coco_data[obj.label].rgb.b,
                                            this->coco_data[obj.label].rgb.g,
                                            this->coco_data[obj.label].rgb.r);
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

            cv::rectangle(bgr, obj.rect, color * 255, 2);

            char text[256];
            sprintf(text, "%s %.1f%%", this->coco_data[obj.label].name.c_str(), obj.prob * 100);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

            cv::Scalar txt_bk_color = color * 0.7 * 255;

            int x = obj.rect.x;
            int y = obj.rect.y + 1;
            // int y = obj.rect.y - label_size.height - baseLine;
            if (y > bgr.rows)
                y = bgr.rows;
            // if (x + label_size.width > bgr.cols)
            // x = bgr.cols - label_size.width;

            cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                          txt_bk_color, -1);

            cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(yolox_ros_cpp::YoloXNode)

