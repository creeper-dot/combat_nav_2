import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_standard_robot_pp_ros2_dir = get_package_share_directory(
        "standard_robot_pp_ros2"
    )
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )
    pkg_wp_map_tools_dir = get_package_share_directory(
        "wp_map_tools"
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    robot_name = LaunchConfiguration("robot_name")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Create our own temporary YAML files that include substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg_standard_robot_pp_ros2_dir,
            "config",
            "standard_robot_pp_ros2.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="combat2025_sentry",
        description="The file name of the robot xmacro to be used",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_waypoint_file_cmd = DeclareLaunchArgument(
        "waypoint_file",
        default_value=os.path.join(pkg_wp_map_tools_dir, "config", "waypoints.yaml"),
        description="Full path to the waypoint YAML file",
    )


    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_pb2025_robot_description_dir,
                        "launch",
                        "robot_description_launch.py",
                    )
                ),
                launch_arguments={
                    "params_file": params_file,
                    "robot_name": robot_name,
                    "use_rviz": use_rviz,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
            # 此节点将 IMU 数据从 lidar 帧转换到 gimbal_yaw 帧
            Node(
                package='imu_transformer',
                executable='imu_transformer_node',
                name='imu_transformer_node',
                parameters=[{
                    'target_frame': 'gimbal_yaw'  # 目标坐标系
                }],
                remappings=[
                    ('/imu_in', '/livox/imu'),          # 输入话题：原始 IMU
                    ('/imu_out', '/imu/transformed'),   # 输出话题：转换后的 IMU（仍无姿态）
                ]
            ),
            # 此节点接收转换后的 IMU 数据，并计算其姿态（四元数）
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_madgwick_node',
                parameters=[{
                    # 基本参数
                    'use_mag': False,                           # 不使用磁力计
                    'world_frame': 'enu',                       # 世界坐标系：东-北-天
                    'fixed_frame': 'odom',                      # 固定参考系
                    'imu_frame': 'gimbal_yaw',                  # IMU 数据所在的坐标系
                    # 滤波器参数
                    'gain': 0.1,                                # 滤波器增益
                    'zeta': 0.0,                                # 阻尼比
                    'publish_tf': False,                        # 不发布 TF
                    'publish_debug_topics': False,              # 不发布调试话题
                    # 重力参数
                    'gravity_constant': 9.81,                   # 重力常数
                    'remove_gravity_acceleration': False,       # 不移除重力加速度
                    # 初始化参数
                    'orientation_stddev': 0.0,                  # 姿态协方差
                    'angular_velocity_stddev': 0.02,            # 角速度协方差
                    'linear_acceleration_stddev': 0.04,         # 加速度协方差
                    # 静态检测参数
                    'stat': {
                        'stationary_time': 5.0,                 # 静止检测时间
                        'acceleration_threshold': 0.1,          # 加速度阈值
                        'magnetic_threshold': 0.0               # 磁力计阈值（未使用）
                    },
                    # 频率参数（根据您的 IMU 频率调整）
                    'frequency': 200.0,                         # 滤波器频率
                    'constant_dt': 0.0                          # 固定时间步长（0表示自动计算）
                    }],
                remappings=[
                    ('/imu/data_raw', '/imu/transformed'),     # 输入：转换后无姿态的IMU
                    ('/imu/data', '/imu/gimbal_yaw') # 输出：最终带姿态的IMU
                ]
            ),
            Node(
                package='wp_map_tools',
                executable='wp_edit_node',
                name='wp_edit_node',
                output='screen',
                respawn=use_respawn,
                # arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'waypoint_file': LaunchConfiguration('waypoint_file'),  # 航点文件路径
                    'frame_id': 'map'  # 航点参考坐标系
                }],
            ),
            Node(
                package='wp_map_tools',
                executable='combat_navi_waypoint',
                name='combat_navi_waypoint',
                output='screen',
                respawn=use_respawn,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'base_waypoint': '2',
                    'resource_waypoint': '1',
                    'low_blood_threshold': 20,
                    'full_blood_threshold': 400,
                    'time_over_threshold': 300,
                }],
            ),
            Node(
                package='wp_map_tools',
                executable='wp_navi_server',
                name='wp_navi_server',
                output='screen',
                respawn=use_respawn,
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_waypoint_file_cmd)

    # Add the actions to launch all of nodes
    ld.add_action(bringup_cmd_group)

    return ld
