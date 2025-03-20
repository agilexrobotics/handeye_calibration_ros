from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    aruco_single_params = {
        'mode': LaunchConfiguration('mode'),
        'min_num': LaunchConfiguration('min_num'),
        'piper_topic': LaunchConfiguration('piper_topic'),
        'marker_topic': LaunchConfiguration('marker_topic'),
    }

    aruco_single = Node(
        package='handeye_calibration_ros', 
        executable='handeye_calibration', 
        name="handeye_calibration",
        output='screen',
        parameters=[aruco_single_params],
    )

    return [aruco_single]


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='eye_in_hand',  #eye_to_hand
        choices=['eye_in_hand', 'eye_to_hand'],
        description='handeye calibration mode'
    )

    min_num_arg = DeclareLaunchArgument(
        'min_num',
        default_value='10',
        description=''
    )

    piper_topic_arg = DeclareLaunchArgument(
        'piper_topic',
        default_value='piper_ctrl_node/end_pose',
        description=''
    )

    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='aruco_single/pose',
        description=''
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(mode_arg)
    ld.add_action(min_num_arg)
    ld.add_action(piper_topic_arg)
    ld.add_action(marker_topic_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

