from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_localization"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    rtabmap_params_file = LaunchConfiguration("rtabmap_params_file")
    fast_lio_config = str(share / "config" / "fast_lio_mid360.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=str(share / "config" / "ekf.yaml"),
            ),
            DeclareLaunchArgument(
                "rtabmap_params_file",
                default_value=str(share / "config" / "rtabmap.yaml"),
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fast_lio",
                output="screen",
                parameters=[fast_lio_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/Odometry", "/fast_lio/odometry"),
                    ("/cloud_registered", "/lio/cloud_registered"),
                    ("/cloud_registered_body", "/lio/cloud_body"),
                    ("/path", "/lio/path"),
                ],
            ),
            Node(
                package="lunar_rover_localization",
                executable="lio_odom_adapter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            Node(
                package="lunar_rover_localization",
                executable="pose_tf_splitter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "approx_sync": True,
                        "queue_size": 20,
                    }
                ],
                remappings=[
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("rgbd_image", "/mapping/rgbd_image"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                arguments=["--delete_db_on_start"],
                parameters=[rtabmap_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("rgbd_image", "/mapping/rgbd_image"),
                    ("scan_cloud", "/lio/cloud_registered"),
                    ("odom", "/odometry/filtered"),
                    ("grid_map", "/map"),
                ],
            ),
        ]
    )
