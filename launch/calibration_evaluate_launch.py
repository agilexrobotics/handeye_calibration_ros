from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    calibration_evaluate_params = {
        'mode': LaunchConfiguration('mode'),
        'transform_file': LaunchConfiguration('transform_file'),
    }

    calibration_evaluate = Node(
        package='handeye_calibration_ros', 
        executable='calibration_evaluate', 
        name="calibration_evaluate",
        output='screen',
        parameters=[calibration_evaluate_params],
        remappings=[  
            # sub
            ('/end_pose_stamped', '/end_pose_stamped'),
            # pub
            ('/pos_cmd', '/pos_cmd'),
        ]
    )

    return [calibration_evaluate]


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='eye_in_hand', 
        choices=['eye_in_hand', 'eye_to_hand'],
        description='calibration evaluate mode'
    )

    transform_file_arg = DeclareLaunchArgument(
        'transform_file',
        default_value='./src/handeye_calibration_ros/result/eye_in_hand.json',
        description=''
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(mode_arg)
    ld.add_action(transform_file_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

