import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory('yolox_ros_cpp')

    yolox_param_yaml = os.path.join(yolox_ros_share_dir, "param", "tiny_openvino.yaml")
    # yolox_param_yaml = os.path.join(yolox_ros_share_dir, "param", "tiny_trtexec.yaml")

    # dog is not exist, download
    url = "https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg"
    dog_jpg = os.path.join(yolox_ros_share_dir, "./", "dog.jpg")
    if not os.path.exists(dog_jpg):
        os.system("wget {} -O {}".format(url, dog_jpg))
    
    image_pub = launch_ros.actions.Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher',
        arguments=[dog_jpg],
        remappings=[
            ('image_raw', 'image_srv'),
        ],
    )

    container = ComposableNodeContainer(
        name='yolox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # image subscriber component
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='using_service_v4l2camera::using_service',
                name='sub_v4l2camera',
                parameters=[{
                    "class_yaml": yolox_param_yaml,
                    "imshow_is_show": True,
                }],
                remappings=[
                    ('image_raw', 'image_srv'),
                ],
            ),

            # YOLOX component
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='yolox_ros_cpp::YoloXNode',
                name='yolox_ros_cpp',
                parameters=[yolox_param_yaml],
            )
        ],
        output='screen',
        arguments= [dog_jpg],
    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription([
        container,
        image_pub,
        # rqt_graph,
    ])
