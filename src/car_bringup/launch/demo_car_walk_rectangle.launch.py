import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # 1. 设置包名和路径
    car_description_pkg = get_package_share_directory('car_description')
    bringup_pkg_path = get_package_share_directory("car_bringup")
    controller_config = os.path.join(bringup_pkg_path, "config", "car_controller.yaml")
    world_file_path = os.path.join(bringup_pkg_path, "worlds", "world.sdf")

    # 2. 声明 Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Whether to launch the RGBD camera nodes'
    )

    # 3. 解析 URDF (使用 xacro)
    xacro_file = os.path.join(car_description_pkg, 'urdf', 'mecanum_car.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_camera:=',LaunchConfiguration('use_camera')])
    
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

    # 5. Gazebo 仿真节点 (-r 代表自动运行，产生时钟)
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
            "-r /mecanum_controller/tf_odometry:=/tf -r /mecanum_controller/reference:=/cmd_vel",
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

    # # 10. RViz 可视化
    rviz_config_file = os.path.join(car_description_pkg, 'rviz', 'description2.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}], 
    )
    
    # 11. 循环矩形圈cmd_vel发布节点
    publish_rectangle_cmdvel_node = Node(
        package='car_nmpc_cbf',
        executable='publish_rectangle_cmdvel_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # 12. 使用 TimerAction 包装该节点，设置延迟时间
    # 比如这里设置 period=5.0，意味着 5 秒后才会启动这个节点
    delayed_rectangle_cmdvel_node = TimerAction(
        period=15.0,					# 延迟时间（单位：秒）
        actions=[publish_rectangle_cmdvel_node]	# 延迟后执行的动作列表
    )
    
    return LaunchDescription([
        use_camera_arg,
        # 仿真基础
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        # 可视化
        node_rviz,
        # 采用事件监听器，等小车成功生成（spawn_entity退出）后，再安全加载控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster, load_mecanum_controller],
            )
        ),
        # 循环矩形圈cmd_vel发布节点
        delayed_rectangle_cmdvel_node
    ])