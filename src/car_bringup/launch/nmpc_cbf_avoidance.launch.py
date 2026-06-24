import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 【新增 A】声明一个外部传入的 Launch 参数
    path_arg = DeclareLaunchArgument(
        'path',
        default_value='s_curve',
        description='Global path shape (e.g., straight, circle, s_curve)'
    )
    # 1. 设置包名和路径
    car_description_pkg = get_package_share_directory('car_description')
    bringup_pkg_path = get_package_share_directory("car_bringup")
    controller_config = os.path.join(bringup_pkg_path, "config", "car_controller2.yaml")
    world_file_path = os.path.join(bringup_pkg_path, "worlds", "world2.sdf")
    
    # 【新增】EKF 融合节点的配置文件路径
    ekf_config_path = os.path.join(bringup_pkg_path, "config", "ekf.yaml")

    # 2. 声明 Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Whether to launch the RGBD camera nodes'
    )

    # 3. 解析 URDF (使用 xacro)
    xacro_file = os.path.join(car_description_pkg, 'urdf', 'mecanum_car.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_camera:=', LaunchConfiguration('use_camera')])
    
    # 4. Robot State Publisher 节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # 5. Gazebo 仿真节点
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # 6. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'car_mecanum'],
    )

    # 7. Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
             "--param-file",
            controller_config,
            ],
    )

    # 8. Mecanum Drive Controller
    load_mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "mecanum_controller",
            "--param-file",
            controller_config,
            "--controller-ros-args",
            "-r /mecanum_controller/reference:=/cmd_vel",
            ],
    )

    # 9. Gazebo 桥接 (时钟 + 激光雷达 + IMU)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/mecanum_car_avoidance_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
        remappings=[('/world/mecanum_car_avoidance_world/clock', '/clock')],
    )

    # 10. RViz 可视化
    rviz_config_file = os.path.join(car_description_pkg, 'rviz', 'description2.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}], 
    )
    
    # 【新增】11. robot_localization (EKF) 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        # 默认 EKF 输出到 /odometry/filtered，这里映射为 /odom 供你的 NMPC 节点使用
        remappings=[('/odometry/filtered', '/odom')]
    )
    
    # 12. NMPC-CBF 控制节点
    nmpc_cbf_node = Node(
        package='car_nmpc_cbf',
        executable='nmpc_cbf_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path': LaunchConfiguration('path')
            }],
    )
    
    # 【新增】13. 使用 TimerAction 延迟 10 秒启动 NMPC 节点
    delayed_nmpc_cbf_node = TimerAction(
        period=10.0,
        actions=[nmpc_cbf_node]
    )
    
    return LaunchDescription([
        use_camera_arg,
        path_arg,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        node_rviz,
        # 事件监听：等小车生成后再加载底层控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster, load_mecanum_controller],
            )
        ),
        # 启动 EKF 融合
        ekf_node,
        # 延迟 10 秒启动高层控制
        delayed_nmpc_cbf_node,
    ])