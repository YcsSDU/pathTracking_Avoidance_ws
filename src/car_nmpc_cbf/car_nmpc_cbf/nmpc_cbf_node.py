import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
import time

from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN

import tf2_ros

import casadi as ca
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class NMPCCBFController(Node):
    def __init__(self):
        super().__init__('nmpc_cbf_controller')
        self.declare_parameter("path", 's_curve')

        # ==========全局参考路径,根据选择不同,可以生成不同的路径=============
        self.internal_path = Path()
        self.path_array = None # 用于存储 Numpy 格式的全局路径 [x, y, theta]
        # 示例：初始化时生成一条 S 曲线，三种："s_curve"、 'straight' 或 'circle'
        self.generate_trajectory(self.get_parameter("path").value, frame_id='odom', num_points=200)
        self.get_logger().info("轨迹生成完成")

        # ==========订阅 LiDAR 话题与数据处理=============
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',    # 替换为你的真实雷达话题
            self.lidar_callback,
            10          # QoS profile depth
        )
        # 存储处理后的最近障碍物坐标，供 MPC/CBF 控制循环使用
        # 格式为: [[x1, y1], [x2, y2], ...] (基于机器人 laser 坐标系)
        self.closest_obstacles = []
        # 聚类参数配置
        self.dbscan_eps = 0.1        # 聚类邻域距离（单位：米），表示相距多近的点算作同一个障碍物
        self.dbscan_min_samples = 3  # 构成一个聚类所需的最少点数（过滤噪点）
        self.max_obstacles = 3       # 提取距离最近的 N 个障碍物给 CBF
        # 设置考虑障碍物点的最大距离
        self.obstacle_max_range = 2.0  # 米，超过这个距离的障碍物点将被忽略
        
        # ================订阅已经融合过传感器数据的里程计===============
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # 当前位姿[x, y, theta] (基于 odom 坐标系)和速度[vx, vy, omega] (基于 base_footprint 坐标系)，供 MPC/CBF 控制循环使用
        self.current_pose = np.zeros(3)
        self.current_twist = np.zeros(3)
        # self.current_pose = np.array([3.0,-0.9, 0.02])
        # self.current_twist = np.zeros(3)

        # ================提取局部参考轨迹:使用get_local_reference_trajectory()方法===============

        # ================MPC参数配置===============
        # MPC 时间窗口参数
        self.Np = 10                  # MPC 预测时域步数 (Prediction Horizon)
        self.Nc = 4                   # MPC 控制时域步数 (Control Horizon)
        self.dt = 0.1                # MPC 采样周期 (Sampling Time, 10Hz)
        self.a_max = 0.5  # 最大线加速度 (m/s^2)
        self.v_max = 0.5  # 全局最高限速 (m/s)
        self.w_max = 0.5  # 最大角速度
        self.d_safe = 0.4            # 膨胀安全距离 (小车半径 + 裕度),小车主体是一个矩形，可以认为是一个0.4米的正方体；裕度再设置为0.2
        # =================CBF参数配置========================
        self.cbf_gamma = 0.2         # CBF 收敛率参数

        # ==================控制指令发布器====================
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # ==================== 初始化 CasADi 求解器 ====================
        w1 = [10.0, 10.0, 1.0]       # 状态跟踪权重: x, y, theta
        w2 = [0.1, 0.1, 0.05]        # 控制输入绝对值惩罚: vx, vy, wz
        w3 = [0.5, 0.5, 0.2]         # 【新增】控制增量惩罚权重: d_vx, d_vy, d_wz
        self._init_casadi_solver(w1, w2, w3)

        # ==================启动控制循环====================
        # 根据你的 dt=0.1，控制频率为 10Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.count = 0  # 用于调试
        # 【新增】首次运行标志位，用于平滑初始化历史控制量
        self.is_first_run = True

        # 用于热启动求解器 (Warm Start)，定义为矩阵形状适配 Opti
        self.u0_guess = np.zeros((3, self.Nc))
        self.x0_guess = np.zeros((3, self.Np + 1))

        # =================雷达外参配置========================
        # 雷达相对于base_link的偏移
        # 解析自 URDF: xyz="-0.0475 0 0.125" rpy="0 0 0"
        self.laser_offset_x = -0.0475
        self.laser_offset_y = 0.0
        # yaw 为 0，后续代码可以直接省略这部分的旋转计算


    def generate_trajectory(self, traj_type, num_points=200, frame_id='odom'):
        """
        根据输入类型在节点类中生成轨迹，并包含正确的朝向（姿态）
        """
        self.get_logger().info(f"正在计算 {traj_type} 轨迹...")
        
        # 1. 准备新的 Path 对象
        self.internal_path = Path()
        self.internal_path.header.stamp = self.get_clock().now().to_msg()
        self.internal_path.header.frame_id = frame_id

        # 2. 根据类型生成坐标点 (x, y)
        points = []
        T= 4.0  # 轨迹总长度（米）
        for i in range(num_points):
            t = i / float(num_points)
            if traj_type == 'straight':
                points.append((t * T, 0.0))
            elif traj_type == 's_curve':
                points.append((t * T, math.sin(t * 2 * math.pi) * 1.0))
            elif traj_type == 'circle':
                angle = t * 2 * math.pi
                points.append((math.cos(angle) * 1.0, math.sin(angle) * 1.0))
            else:
                self.get_logger().error(f"抱歉，不支持该轨迹类型: {traj_type}")
                return

        # 3. 计算位姿并写入内部变量
        for i in range(num_points):
            # 计算切线方向 (当前点到下一个点的向量)
            if i < num_points - 1:
                dx = points[i+1][0] - points[i][0]
                dy = points[i+1][1] - points[i][1]
            else:
                # 终点沿用前向向量
                dx = points[i][0] - points[i-1][0]
                dy = points[i][1] - points[i-1][1]
            
            # 将二维偏航角转换为四元数
            yaw = math.atan2(dy, dx)
            quat = R.from_euler('z', yaw).as_quat()

            # 填充 ROS 消息
            pose = PoseStamped()
            pose.header.stamp = self.internal_path.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = points[i][0]
            pose.pose.position.y = points[i][1]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            # 存入节点类属性
            self.internal_path.poses.append(pose)
        # 【新增】将路径存为 Numpy 数组，加速后续 MPC 计算
        path_list = []
        for pose in self.internal_path.poses:
            # 四元数转 yaw
            q = pose.pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
            path_list.append([pose.pose.position.x, pose.pose.position.y, yaw])
            self.get_logger().info(f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, yaw={yaw:.3f}")
        self.path_array = np.array(path_list)
        self.get_logger().info(f"生成完毕！当前 self.internal_path 包含 {len(self.internal_path.poses)} 个点。")

    def lidar_callback(self, msg: LaserScan):
        """
        处理单线激光雷达数据的回调函数
        """
        # 1. 获取原始距离数组并生成对应的角度数组
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # 2. 数据清洗 (过滤掉 NaN, Inf 以及超出测距范围的无效点)
        min_range = max(float(msg.range_min), 0.0)
        max_range = min(float(msg.range_max), self.obstacle_max_range)
        valid_mask = (
            np.isfinite(ranges) & 
            (ranges > min_range) & 
            (ranges < max_range)
        )
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        # 如果当前“关注圈”内没有任何障碍物，直接清空并返回，节省后续所有计算资源
        if len(valid_ranges) == 0:
            self.closest_obstacles = []
            return

        # 3. 极坐标 (r, theta) 转换为笛卡尔坐标 (x, y) - 基于雷达坐标系
        # x = r * cos(theta), y = r * sin(theta)
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y))  # 形状: (N, 2)

        # 4. 使用 DBSCAN 算法对二维点云进行聚类
        # labels 数组包含了每个点所属的聚类 ID，-1 表示噪声
        """
        clustering:DBSCAN 模型对象
        clustering.labels_:每个样本的聚类标签（一维数组）
        """
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        # labels[i] 表示第 i 个点属于哪个簇
        # 特殊值 -1：表示噪声点（离群点），不属于任何簇
        # 编号规则：有效簇从 0 开始递增（0, 1, 2...）
        labels = clustering.labels_
        # 所有不重复的标签集合
        unique_labels = set(labels)
        cluster_closest_points = []

        # 5. 遍历每个聚类，提取距离机器人（原点 0,0）最近的点
        for label in unique_labels:
            if label == -1:
                continue  # 忽略被判定为噪声的游离点
                
            # 提取属于当前聚类的所有点
            cluster_points = points[labels == label]
            
            # 计算这些点到laser中心(0,0)的欧式距离平方 (省去开根号计算更快)
            distances_sq = cluster_points[:, 0]**2 + cluster_points[:, 1]**2
            
            # 找到距离最近的点的索引
            min_idx = np.argmin(distances_sq)
            closest_point = cluster_points[min_idx]
            min_dist = np.sqrt(distances_sq[min_idx])
            
            cluster_closest_points.append((min_dist, closest_point))

        # 6. 按距离对所有障碍物进行排序，并只截取前 N 个最危险的障碍物
        cluster_closest_points.sort(key=lambda item: item[0])
        top_n_obstacles = [item[1].tolist() for item in cluster_closest_points[:self.max_obstacles]]
        
        # 更新类变量，你的控制 Timer Callback 可以直接读取这个变量来构建 CBF 约束
        self.closest_obstacles = top_n_obstacles
        
        # 调试输出（可以在终端验证）
        if self.closest_obstacles:
           self.get_logger().info(f"Detected {len(self.closest_obstacles)} obstacles: {self.closest_obstacles}")

    def odom_callback(self, msg: Odometry):
        # 提取位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 四元数转偏航角 (Quaternion to Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = np.array([x, y, theta])
        
        # 提取当前体坐标系速度 (Mecanum控制直接使用 vx, vy, wz)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.current_twist = np.array([vx, vy, wz])
        
    def get_local_reference_trajectory(self):
        if self.path_array is None or len(self.path_array) == 0:
            return None
            
        # A. 寻找当前距离小车最近的全局路径点索引
        cur_x, cur_y = self.current_pose[0], self.current_pose[1]
        dx = self.path_array[:, 0] - cur_x
        dy = self.path_array[:, 1] - cur_y
        distances = np.sqrt(dx**2 + dy**2)
        closest_idx = np.argmin(distances)
        
        # B. 沿路径计算离散点之间的累计距离（用于插值）
        # 只计算从最近点开始往后的路径长度
        remaining_path = self.path_array[closest_idx:]
        if len(remaining_path) < 2:
            # 如果快到终点了，直接用终点填充整个预测时域
            return np.tile(self.path_array[-1], (self.Np, 1))
        # 每段路径长度 = sqrt((x2-x1)^2 + (y2-y1)^2)，累计长度 = 前一段累计长度 + 当前段长度   
        seg_lengths = np.sqrt(np.diff(remaining_path[:, 0])**2 + np.diff(remaining_path[:, 1])**2)
        # cum_lengths 是长度为 M 的数组，表示每个路径点距离closest_idx的累计距离，cum_lengths[0] = 0, cum_lengths[1] = seg_lengths[0], cum_lengths[2] = seg_lengths[0] + seg_lengths[1], ...
        cum_lengths = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        
        # C. 基于时间窗口和目标速度，计算预测时域内,匀速运动假设下，未来每个时刻车辆应该到达的累计里程
        # 获取小车当前真实线速度合速度
        v_current = np.hypot(self.current_twist[0], self.current_twist[1])
        # 计算距离终点的剩余路径总长度
        distance_to_goal = cum_lengths[-1]
        # 动态规划预测时域内的速度序列
        v_seq = np.zeros(self.Np)
        v_step = v_current
        for i in range(self.Np):
            # 考虑加速度限制 (平滑起步)
            v_step = min(v_step + self.a_max * self.dt, self.v_max)
            # 考虑终点减速 (非常简单的比例减速控制，防止冲过头)
            # 假设距离终点 1.0 米开始减速
            if distance_to_goal < 1.0:
                v_decel = max(distance_to_goal * self.v_max, 0.0) # 距离越近，限速越低
                v_step = min(v_step, v_decel) 
            v_seq[i] = v_step
            # 模拟小车在这一步走完后，距离终点变近了
            distance_to_goal -= v_step * self.dt
        # 基于动态速度序列，计算累计期望距离 (这就是插值所需的 target_distances)
        target_distances = np.cumsum(v_seq * self.dt)
            
        # D. 使用线性插值获取每个时间步的 [x, y, theta]
        ref_x = np.interp(target_distances, cum_lengths, remaining_path[:, 0], right=remaining_path[-1, 0])
        ref_y = np.interp(target_distances, cum_lengths, remaining_path[:, 1], right=remaining_path[-1, 1])
        ref_theta = np.interp(target_distances, cum_lengths, remaining_path[:, 2], right=remaining_path[-1, 2])
        
        # 拼接成形状为 (N, 3) 的矩阵
        local_traj = np.column_stack((ref_x, ref_y, ref_theta))
        return local_traj

    def _init_casadi_solver(self, w1, w2, w3):
        """在节点启动时构建一次符号化计算图 (基于 ca.Opti)"""
        self.opti = ca.Opti()
        
        # 1. 定义优化变量 (决策变量 X, U)
        # X: 状态量 (3行, Np+1列) -> [x, y, theta]
        # U: 控制量 (3行, Nc列) -> [vx, vy, wz]
        self.X = self.opti.variable(3, self.Np + 1)
        self.U = self.opti.variable(3, self.Nc)
        
        # 2. 定义参数变量 (每次 control_loop 中动态更新的值)
        self.p_current_pose = self.opti.parameter(3)             # 当前状态 [x, y, theta]
        self.p_local_traj = self.opti.parameter(3, self.Np)      # 预测时域局部参考轨迹
        self.p_obs = self.opti.parameter(2, self.max_obstacles)  # 障碍物坐标矩阵 (2行: x, y)
        self.p_last_u = self.opti.parameter(3)                   # 【新增】上一时刻的实际控制量
        
        # 3. 定义运动学模型
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        
        vx = ca.SX.sym('vx'); vy = ca.SX.sym('vy'); wz = ca.SX.sym('wz')
        controls = ca.vertcat(vx, vy, wz)
        
        rhs = ca.vertcat(
            vx * ca.cos(theta) - vy * ca.sin(theta),
            vx * ca.sin(theta) + vy * ca.cos(theta),
            wz
        )
        # 运动学方程 f(x, u)
        f = ca.Function('f', [states, controls], [rhs])
        
        # 4. 定义权重矩阵
        Q = ca.diag(w1)
        R = ca.diag(w2)
        R_delta = ca.diag(w3)  # 【新增】控制增量权重矩阵
        
        # 5. 构建 Cost 和 约束
        cost = 0.0
        
        # 初始状态等式约束
        self.opti.subject_to(self.X[:, 0] == self.p_current_pose)
        
        for k in range(self.Np):
            st = self.X[:, k]
            # 超出控制时域后，控制量保持不变
            con = self.U[:, k] if k < self.Nc else self.U[:, self.Nc - 1]
            st_ref = self.p_local_traj[:, k]
            
            # (1) 累加代价函数
            err = st - st_ref
            cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([con.T, R, con])
            # 【新增】控制增量代价计算
            if k < self.Nc:
                if k == 0:
                    delta_con = con - self.p_last_u  # 第一步相对于上一周期实际指令
                else:
                    delta_con = con - self.U[:, k-1] # 后续步相对于时域内的前一步
                cost += ca.mtimes([delta_con.T, R_delta, delta_con])

            # (2) 运动学多重打靶 (Multiple Shooting) 等式约束
            st_next_euler = st + self.dt * f(st, con)
            self.opti.subject_to(self.X[:, k+1] == st_next_euler)
            
            # (3) CBF 避障不等式约束
            for i in range(self.max_obstacles):
                obs_x = self.p_obs[0, i]
                obs_y = self.p_obs[1, i]
                
                # 当前步和预测下一步的距离平方
                hk = (st[0] - obs_x)**2 + (st[1] - obs_y)**2 - self.d_safe**2
                hk_next = (st_next_euler[0] - obs_x)**2 + (st_next_euler[1] - obs_y)**2 - self.d_safe**2
                
                # 离散 CBF 约束: h(k+1) - (1-gamma)*h(k) >= 0
                cbf_val = hk_next - (1.0 - self.cbf_gamma) * hk
                self.opti.subject_to(cbf_val >= 0)
                
        # 6. 控制量上下界约束
        """
        self.opti.bounded(lb, expr, ub)
        生成双向不等式约束：lb ≤ expr ≤ ub
        """
        self.opti.subject_to(self.opti.bounded(-self.v_max, self.U[0, :], self.v_max))  # vx限制
        self.opti.subject_to(self.opti.bounded(-self.v_max, self.U[1, :], self.v_max))  # vy限制
        self.opti.subject_to(self.opti.bounded(-self.w_max, self.U[2, :], self.w_max))  # wz限制
        
        # 7. 设置优化目标
        self.opti.minimize(cost)
        
        # 8. 求解器配置
        opts = {
            'ipopt.max_iter': 50,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-4,
        }
        self.opti.solver('ipopt', opts)

    def control_loop(self):
        self.count += 1
        # 确保全局路径数据已就绪
        if self.path_array is None:
            self.get_logger().warn("Waiting for global path...", throttle_duration_sec=2.0)
            return
        
        # 提取当前时间窗口内的局部参考轨迹
        local_trajectory = self.get_local_reference_trajectory()
        # 调试输出当前的位姿
        if self.count == 5:
            print("current_pose:", self.current_pose, "\n", "current_twist:", self.current_twist)
            print("local_trajectory:", local_trajectory)
            self.count = 0
        if local_trajectory is None:
            return

        # 【新增优化】如果是第一次执行控制循环，将历史指令对齐为当前里程计的真实反馈速度
        # 从而彻底消除突变隐患，保证运动中的“热启动”平滑过渡
        if self.is_first_run:
            self.last_u_cmd = self.current_twist.copy()
            self.is_first_run = False
            self.get_logger().info(f"MPC控制量完成首次对齐，初始速度基准为: {self.last_u_cmd}")
        
        # =======================================================
        # 至此，你已经准备好了 MPC + CBF 控制器所需的全部输入数据：
        # 1. 当前状态（小车位姿）：self.current_pose  -> [x, y, theta]
        # 2. 当前输入（小车速度）：self.current_twist -> [vx, vy, omega]
        # 3. 未来参考轨迹（时域）：local_trajectory   -> 形状为 (N, 3) 的矩阵
        # 4. 局部关键障碍物坐标：   self.closest_obstacles -> 最危险的 N 个障碍物点
        # =======================================================

        # ==================== 障碍物坐标系转换 ====================
        cx, cy, ctheta = self.current_pose
        obs_odom = []
        cos_c = math.cos(ctheta)
        sin_c = math.sin(ctheta)
        
        for obs in self.closest_obstacles:
            lx, ly = obs[0], obs[1]
            bx = self.laser_offset_x + lx
            by = self.laser_offset_y + ly
            ox_odom = cx + bx * cos_c - by * sin_c
            oy_odom = cy + bx * sin_c + by * cos_c
            obs_odom.extend([ox_odom, oy_odom])
            
        # 填充缺少的障碍物坐标到极远处
        while len(obs_odom) < 2 * self.max_obstacles:
            obs_odom.extend([999.0, 999.0])
            
        # 将一维的障碍物坐标转换为 (2, max_obstacles) 形状的 Numpy 数组，以匹配 Opti 参数维度
        obs_array = np.array(obs_odom).reshape(self.max_obstacles, 2).T

        # ==================== Opti 参数投喂 ====================
        self.opti.set_value(self.p_current_pose, self.current_pose)
        self.opti.set_value(self.p_local_traj, local_trajectory.T)
        self.opti.set_value(self.p_obs, obs_array)
        self.opti.set_value(self.p_last_u, self.last_u_cmd)  # 【新增】投喂上一时刻实际速度

        # 投喂上一周期的热启动猜测值
        self.opti.set_initial(self.X, self.x0_guess)
        self.opti.set_initial(self.U, self.u0_guess)

        # ==================== 调用求解器 ====================
        try:
            sol = self.opti.solve()
            sol_x = sol.value(self.X)  
            sol_u = sol.value(self.U)  
            
            # 【优化】执行 Horizon Shift。抛弃当前已被执行的第0步，末尾复制最后一步填充
            # 控制量 U 形状为 (3, Nc)，移位后供下一周期热启动使用
            self.u0_guess = np.hstack((sol_u[:, 1:], sol_u[:, -1:]))
            # 状态量 X 形状为 (3, Np+1)，同步进行移位
            self.x0_guess = np.hstack((sol_x[:, 1:], sol_x[:, -1:]))
            
            # 提取当前步的最优控制量
            vx_cmd, vy_cmd, wz_cmd = sol_u[:, 0]
            
            # 更新历史控制量，供下一周期计算控制增量使用
            self.last_u_cmd = np.array([vx_cmd, vy_cmd, wz_cmd])

        except RuntimeError as e:
            self.get_logger().error(f"Solver failed: {e}")
            self.stop_robot()
            # 即使求解失败，也让热启动值平滑退化，防止下一周期彻底崩掉
            self.u0_guess = np.zeros((3, self.Nc))
            self.x0_guess = np.zeros((3, self.Np + 1))
            self.last_u_cmd = np.zeros(3)
            return
        # ==================== 发布指令 ====================
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        
        cmd_msg.twist.linear.x = float(vx_cmd)
        cmd_msg.twist.linear.y = float(vy_cmd)
        cmd_msg.twist.angular.z = float(wz_cmd)
        
        self.cmd_pub.publish(cmd_msg)

    def stop_robot(self):
        """安全停机函数：清空所有轴向速度"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 确保 X、Y、Z 全部归零
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)
        self.get_logger().warn("🛑 触发麦轮安全刹车：已下发全轴零速指令！")

def main():
    rclpy.init()
    node = NMPCCBFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C 中断信号。")
    except Exception as e:
        node.get_logger().error(f"节点异常崩溃: {e}")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()