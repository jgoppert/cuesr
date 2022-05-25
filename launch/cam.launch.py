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
        SetEnvironmentVariable(name='GSCAM_CONFIG', value="v4l2src ! image/jpeg,width=640,height=480,framerate=10/1 ! jpegdec ! videoconvert"),
        # SetEnvironmentVariable(name='GSCAM_CONFIG', value="v4l2src do-timestamp=true ! width=480,height=320,framerate=25/1 ! videoconvert"),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('cuesr') + '/config/camera.yaml'),
        Node(
            package='gscam',
            executable='gscam_node',
            namespace='thermal',
            parameters=[
                {'camera_info_url': 'package://cuesr/config/thermal_5_18_22.yaml'},
                {'frame_id': 'thermal_frame'},
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera') ,'/launch/rs_launch.py']),
            launch_arguments={
                'enable_pointcloud': 'true',
                'align_depth': 'true',
                'depth_fps': '6.0',
                'depth_width': '848',
                'depth_height': '480',
                'color_fps': '6.0',
                'color_width': '640',
                'color_height': '480',
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
           arguments=["0", "0.05", "0", "0", "-0.02", "0.08", "camera_depth_optical_frame", "thermal_frame"]
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
            #prefix=['xterm -e gdb --args'],
            name='register',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Registers the depth image in the thermal camera frame
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::RegisterNode',
                    name='register_node',
                    remappings=[
                        #Subscribed
                        ('rgb/camera_info', '/thermal/camera/camera_info'),
                        ('rgb/image_rect_color', '/thermal/camera/image_rect'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('depth/image_rect', '/camera/depth/image_rect_raw'),
                        ('points', '/thermal/camera/points'),
                        #Published
                        ('depth_registered/image_rect', '/thermal/camera/depth_registered/image_rect'),
                        ('depth_registered/image_rect/compressed', '/thermal/camera/depth_registered/image_rect/compressed'),
                        ('depth_registered/image_rect/compressedDepth', '/thermal/camera/depth_registered/image_rect/compressedDepth'),
                        ('depth_registered/camera_info', '/thermal/camera/depth_registered/camera_info'),
                    ]
                ),
            ],
            output='screen',
        ),

        ComposableNodeContainer(
            #prefix=['xterm -e gdb --args'],
            name='pointcloud',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #Takes the registered depth and thermal images and creates a 3D point cloud
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[
                        #Subscribed
                        ('rgb/camera_info', '/thermal/camera/camera_info'),
                        ('rgb/image_rect_color', '/thermal/camera/image_rect'),
                        ('depth_registered/image_rect', '/thermal/camera/depth_registered/image_rect'),                       
                        ('depth_registered/image_rect/compressed', '/thermal/camera/depth_registered/image_rect/compressed'),
                        ('depth_registered/image_rect/compressedDepth', '/thermal/camera/depth_registered/image_rect/compressedDepth'),
                        #Published
                        ('points', '/thermal/camera/points'),
                    ]
                ),
            ],
            output='screen',
        ),

        # Node(
          # package='rqt_gui',
          # executable='rqt_gui',
        # ),

        Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', get_package_share_directory('cuesr') + '/config/pointcloud.rviz']
        ),
    ])
