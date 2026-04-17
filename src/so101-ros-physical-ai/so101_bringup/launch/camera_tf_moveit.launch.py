from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # This TF layout matches the un-prefixed URDF/SRDF used by MoveIt in this workspace.
    # The overhead camera transform below is intentionally left as a manually
    # tunable estimate. For the current workspace, this is more reliable than
    # the last failed hand-eye calibration result.
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_x", default_value="-0.15"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.80"),
            DeclareLaunchArgument("camera_roll", default_value="3.141592653589793"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            # Overhead camera pose in the world frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_overhead_cam_moveit",
                arguments=[
                    "--x", camera_x,
                    "--y", camera_y,
                    "--z", camera_z,
                    "--roll", camera_roll,
                    "--pitch", camera_pitch,
                    "--yaw", camera_yaw,
                    # Use base_link as parent to avoid needing an explicit world->base_link TF.
                    "--frame-id", "base_link",
                    "--child-frame-id", "cam_overhead",
                ],
            ),
            # Wrist camera pose relative to the end-effector link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_wrist_cam_moveit",
                arguments=[
                    "--x", "0.00",
                    "--y", "0.0",
                    "--z", "-0.02",
                    "--roll", "-1.57",
                    "--pitch", "0.0",
                    "--yaw", "-1.57",
                    "--frame-id", "moving_jaw_so101_v1_link",
                    "--child-frame-id", "cam_wrist",
                ],
            ),
        ]
    )
