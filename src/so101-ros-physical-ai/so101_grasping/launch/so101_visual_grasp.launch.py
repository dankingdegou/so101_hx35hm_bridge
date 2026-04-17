from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_topic = LaunchConfiguration("pose_topic")
    execute = LaunchConfiguration("execute")
    wait_pose_timeout_s = LaunchConfiguration("wait_pose_timeout_s")
    pose_sample_window_s = LaunchConfiguration("pose_sample_window_s")
    pose_sample_count = LaunchConfiguration("pose_sample_count")
    max_pose_spread_m = LaunchConfiguration("max_pose_spread_m")
    post_rest_settle_s = LaunchConfiguration("post_rest_settle_s")
    grasp_x_offset_m = LaunchConfiguration("grasp_x_offset_m")
    grasp_y_offset_m = LaunchConfiguration("grasp_y_offset_m")
    grasp_z_offset_m = LaunchConfiguration("grasp_z_offset_m")
    pregrasp_offset_m = LaunchConfiguration("pregrasp_offset_m")
    planning_time_s = LaunchConfiguration("planning_time_s")
    vel_scaling = LaunchConfiguration("vel_scaling")
    acc_scaling = LaunchConfiguration("acc_scaling")
    position_only = LaunchConfiguration("position_only")
    go_to_rest_before_grasp = LaunchConfiguration("go_to_rest_before_grasp")
    use_marker_orientation = LaunchConfiguration("use_marker_orientation")
    use_rpy = LaunchConfiguration("use_rpy")
    qx = LaunchConfiguration("qx")
    qy = LaunchConfiguration("qy")
    qz = LaunchConfiguration("qz")
    qw = LaunchConfiguration("qw")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_topic", default_value="/vision/red_block/pose_base"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("wait_pose_timeout_s", default_value="10.0"),
            DeclareLaunchArgument("pose_sample_window_s", default_value="0.35"),
            DeclareLaunchArgument("pose_sample_count", default_value="5"),
            DeclareLaunchArgument("max_pose_spread_m", default_value="0.03"),
            DeclareLaunchArgument("post_rest_settle_s", default_value="1.0"),
            DeclareLaunchArgument("grasp_x_offset_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_y_offset_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="-0.015"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("planning_time_s", default_value="8.0"),
            DeclareLaunchArgument("vel_scaling", default_value="0.15"),
            DeclareLaunchArgument("acc_scaling", default_value="0.15"),
            DeclareLaunchArgument("position_only", default_value="true"),
            DeclareLaunchArgument("go_to_rest_before_grasp", default_value="true"),
            DeclareLaunchArgument("use_marker_orientation", default_value="false"),
            DeclareLaunchArgument("use_rpy", default_value="false"),
            DeclareLaunchArgument("qx", default_value="0.0"),
            DeclareLaunchArgument("qy", default_value="1.0"),
            DeclareLaunchArgument("qz", default_value="0.0"),
            DeclareLaunchArgument("qw", default_value="0.0"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            Node(
                package="so101_grasping",
                executable="so101_visual_grasp",
                output="screen",
                remappings=[("joint_states", "/follower/joint_states")],
                parameters=[
                    {
                        "pose_topic": pose_topic,
                        "execute": execute,
                        "wait_pose_timeout_s": wait_pose_timeout_s,
                        "pose_sample_window_s": pose_sample_window_s,
                        "pose_sample_count": pose_sample_count,
                        "max_pose_spread_m": max_pose_spread_m,
                        "post_rest_settle_s": post_rest_settle_s,
                        "grasp_x_offset_m": grasp_x_offset_m,
                        "grasp_y_offset_m": grasp_y_offset_m,
                        "grasp_z_offset_m": grasp_z_offset_m,
                        "pregrasp_offset_m": pregrasp_offset_m,
                        "planning_time_s": planning_time_s,
                        "vel_scaling": vel_scaling,
                        "acc_scaling": acc_scaling,
                        "position_only": position_only,
                        "go_to_rest_before_grasp": go_to_rest_before_grasp,
                        "use_marker_orientation": use_marker_orientation,
                        "use_rpy": use_rpy,
                        "qx": qx,
                        "qy": qy,
                        "qz": qz,
                        "qw": qw,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                    }
                ],
            ),
        ]
    )
