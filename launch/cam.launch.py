import launch
from launch import LaunchDescription
from launch_ros.actions import Node
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
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera') ,'/launch/rs_launch.py']),
            launch_arguments={
                'align_depth': 'True',
                'camera_info_url': 'package://cuesr/config/depth_5_19_22.yaml',
                }.items(),
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

        Node(
           package='rqt_gui',
           executable='rqt_gui',
        ),

    ])
