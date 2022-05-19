import launch
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='GSCAM_CONFIG', value="v4l2src do-timestamp=true ! image/jpeg,width=640,height=480,framerate=10/1 ! jpegdec ! videoconvert"),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('cuesr') + '/config/camera.yaml'),
        Node(
            package='gscam',
            executable='gscam_node',
            namespace='thermal',
            parameters=[
                {'camera_info_url': 'package://cuesr/config/thermal_5_18_22.yaml'},
                {'frame_id': 'camera_depth_optical_frame'},
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera') ,'/launch/rs_launch.py']),
            launch_arguments={
                'pointcloud.enable': 'True',
                'align_depth': 'True',
                'camera_info_url': 'package://cuesr/config/depth_5_19_22.yaml',
                }.items(),
        ),

        Node(
           package='tf2_ros',
           output='screen',
           executable='static_transform_publisher',
           arguments=["0", "0", "0", "0", "0", "0", "map", "camera_link"]
        ),

        Node(
           package='tf2_ros',
           output='screen',
           executable='static_transform_publisher',
           arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_frame"]
        ),
 
 
        Node(
           package='image_proc',
           output='screen',
           executable='image_proc',
           namespace='thermal/camera',
           remappings=[
               ("image", "image_raw")
           ],
        ),

        ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[
                        ('rgb/camera_info', '/thermal/camera/camera_info'),
                        ('rgb/image_rect_color', '/thermal/camera/image_rect'),
                        ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_rect_raw'),
                        ('points', '/thermal/camera/points')
                    ]
                ),
            ],
            output='screen',
        ),

        Node(
           package='rqt_gui',
           executable='rqt_gui',
        ),

    ])
