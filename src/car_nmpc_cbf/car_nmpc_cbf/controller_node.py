import rclpy
import numpy as np
import math

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import casadi as ca
except ImportError:
    ca = None


class nmpc_cbf_controller(Node):
    def __init__(self):
        super().__init__('nmpc_cbf_controller')

        #参数设置
        self.declare_parameter("odom_topic", "/mecanum_controller/odometry") #里程计数据订阅的话题
        self.declare_parameter("path_topic", "/plan_path")   #全局参考路径订阅的话题
        self.declare_parameter("cmd_topic", "/mecanum_controller/cmd_vel") #控制命令发布的话题
        self.declare_parameter("scan_topic", "/scan") #激光雷达数据订阅的话题
        self.declare_parameter("imu_topic", "/imu/data") #IMU数据订阅的话题

        self.declare_parameter("control_rate", 10.0) #控制频率，单位Hz
        self.declare_parameter("dt", 0.1) #控制周期时间间隔，单位秒

        self.declare_parameter("prediction_horizon", 10) #预测时域长度，即NMPC问题中考虑的未来时间步数
        self.declare_parameter("control_horizon", 3)    # 控制时域长度，即实际优化求解的控制步数，默认为0表示自动计算为预测时域长度的20%，至少为1
        self.declare_parameter("control_horizon_fraction", 0.2) #当control_horizon参数设置为0或负数时，控制时域长度将自动计算为预测时域长度的这个比例，至少为1

        self.declare_parameter("robot_radius", 0.35) #小车的半径，单位米，例如0.35米
        self.declare_parameter("safety_margin", 0.15)#安全边距，单位米，例如0.15米，这个值会加在障碍物的半径上，作为CBF约束中的安全距离，确保小车与障碍物之间保持足够的安全距离
        #最大速度限制，单位米/秒和弧度/秒，例如max_vx=0.8表示小车在x方向的最大线速度为0.8米/秒，max_wz=1.5表示小车的最大角速度为1.5弧度/秒
        self.declare_parameter("max_vx", 0.8)
        self.declare_parameter("max_vy", 0.8)
        self.declare_parameter("max_wz", 1.5)

        self.declare_parameter("use_lidar_obstacles", True) #是否使用激光雷达数据来提取障碍物信息，作为NMPC问题的约束条件之一，如果设置为False，则只使用静态障碍物信息（例如通过参数设置的obstacles列表） --- IGNORE ---
        self.declare_parameter("obstacle_sector_half_angle", 1.0472)   #对应60度,以车朝向为中线，左右各60°
        self.declare_parameter("obstacle_max_range", 4.0)   #只关注以小车为中心，这个范围内的障碍物点，单位米
        self.declare_parameter("max_obstacle_points", 8)    #在处理激光雷达数据时，筛选出的障碍物点的最大数量，单位个，例如10个，这样可以限制NMPC问题的复杂度，避免过多障碍物点导致求解困难或者不稳定
        self.declare_parameter("obstacle_min_separation", 0.10)  #在处理激光雷达数据时，筛选出的障碍物点之间至少要有这个距离，单位米，例如0.20米，这样可以避免选到过于密集的点，导致NMPC问题过于复杂或者不稳定
        #激光雷达数据最晚多久之前的还算新鲜，单位秒，如果超过这个时间没有收到新的激光雷达数据，就认为数据过期了，NMPC将不再使用基于激光雷达的CBF约束，以避免使用过时的障碍物信息导致不安全的行为
        self.declare_parameter("scan_timeout", 0.5)
         #激光雷达相对于小车中心的偏移位置和朝向，单位米和弧度，需要根据激光雷达实际安装位置确定，通常激光雷达安装在小车前部，所以x偏移为正，y偏移和yaw根据实际安装情况设置
        self.declare_parameter("laser_offset_x", -0.0475)
        self.declare_parameter("laser_offset_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)  
        #CBF相关参数
        self.declare_parameter("cbf_gamma", 0.45)
        self.declare_parameter("cbf_slack_weight", 10000.0)

        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("imu_timeout", 0.5)
        self.declare_parameter("imu_yaw_weight", 0.75)

        self.declare_parameter("obstacles", [2.0, 2.0, 0.4])  #预设的静态障碍物信息，格式为[(obs_x, obs_y, obs_r), ...]，其中obs_x和obs_y是障碍物的坐标，obs_r是障碍物的半径，这些信息可以用来构建CBF约束，确保小车避开这些静态障碍物

        #一些设置参数
        self.dt = float(self.get_parameter("dt").value)  #控制周期时间间隔，单位秒
        self.np_horizon = int(self.get_parameter("prediction_horizon").value)  #预测时域长度
        self.nc_horizon = self._read_control_horizon()     #控制时域长度，实际控制步数，默认为预测时域长度的20%，至少为1

        #激光雷达相关参数设置
        self.use_lidar_obstacles = bool(    # 是否使用激光雷达数据来提取障碍物信息，作为NMPC问题的约束条件之一，如果设置为False，则只使用静态障碍物信息（例如通过参数设置的obstacles列表） --- IGNORE ---
            self.get_parameter("use_lidar_obstacles").value
        )
        self.obstacle_sector_half_angle = float( #以车朝向为中线，左右各多少度的扇形区域内的障碍物点才会被考虑，单位弧度，例如1.0472对应60度
            self.get_parameter("obstacle_sector_half_angle").value
        )
        self.obstacle_max_range = float(     #在处理激光雷达数据时，只关注这个范围内的障碍物点，单位米
            self.get_parameter("obstacle_max_range").value
        )
        self.max_obstacle_points = int(      #在处理激光雷达数据时，筛选出的障碍物点的最大数量，单位个，例如10个，这样可以限制NMPC问题的复杂度，避免过多障碍物点导致求解困难或者不稳定
            self.get_parameter("max_obstacle_points").value
        )
        self.obstacle_min_separation = float(   #在处理激光雷达数据时，筛选出的障碍物点之间至少要有这个距离，单位米，例如0.20米，这样可以避免选到过于密集的点，导致NMPC问题过于复杂或者不稳定
            self.get_parameter("obstacle_min_separation").value
        )
        
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        #激光雷达相对于小车中心的偏移位置和朝向，单位米和弧度
        self.laser_offset_x = float(self.get_parameter("laser_offset_x").value)
        self.laser_offset_y = float(self.get_parameter("laser_offset_y").value)
        self.laser_yaw = float(self.get_parameter("laser_yaw").value)
        #在处理激光雷达数据时，筛选出的障碍物点的列表，格式为[(x_body, y_body), ...]，表示障碍物点在小车本体坐标系（Body Frame）下的坐标
        self.lidar_obstacle_points_body = []
        
        #最后一次收到激光雷达数据的时间戳，用于判断数据是否过期，单位秒
        self.last_scan_time = None


        self.robot_radius = float(self.get_parameter("robot_radius").value)  #小车的半径，单位米，例如0.35米
        self.safety_margin = float(self.get_parameter("safety_margin").value) #安全边距，单位米，例如0.15米，这个值会加在障碍物的半径上，作为CBF约束中的安全距离，确保小车与障碍物之间保持足够的安全距离
        #最大速度限制，单位米/秒和弧度/秒，例如max_vx=0.8表示小车在x方向的最大线速度为0.8米/秒，max_wz=1.5表示小车的最大角速度为1.5弧度/秒
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.static_obstacles = self._read_obstacles() #从参数中读取预设的静态障碍物信息，格式为[(obs_x, obs_y, obs_r), ...]，其中obs_x和obs_y是障碍物的坐标，obs_r是障碍物的半径，这些信息可以用来构建CBF约束，确保小车避开这些静态障碍物
        
        
        self._reported_no_casadi = False             #用于记录是否已经警告过没有安装casadi，避免重复警告
        self.last_u_sequence = None          #求解成功后保存完整预测时域长度的控制序列
        self.ref_path = []         #全局参考路径点列表
        self.current_state = None     #当前状态，格式为[x, y, yaw]

        #一些CBF相关的参数
        self.cbf_gamma = float(self.get_parameter("cbf_gamma").value)
        self.cbf_slack_weight = float(
            self.get_parameter("cbf_slack_weight").value
        )

        #IMU和odom数据融合相关参数
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout").value)
        self.imu_yaw_weight = float(self.get_parameter("imu_yaw_weight").value)
        
        self.latest_imu_yaw = None
        self.latest_imu_time = None
        self.imu_yaw_offset = None
        

        
        #控制命令发布器
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_topic").value,
            10,
        )

        #定时器，用于发布命令
        control_rate = max(
            float(self.get_parameter("control_rate").value),
            1.0,
        )
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        #订阅2D激光雷达数据
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )
        #订阅里程计数据
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        #订阅IMU数据
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.imu_callback,
            10,
        )
        #订阅全局参考路径数据
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("path_topic").value,
            self.path_callback,
            10,
        )

        self.get_logger().info(
            "NMPC horizon N=%d, control horizon Nc=%d"
            % (self.np_horizon, self.nc_horizon)
        )

    def control_loop(self):
        """控制循环的回调函数"""
        #通过求解NMPC+CBF问题得到控制命令
        cmd_values = self.solve_nmpc_cbf()
        #将求解得到的控制命令发布出去
        cmd = Twist()
        cmd.linear.x = float(cmd_values[0])
        cmd.linear.y = float(cmd_values[1])
        cmd.angular.z = float(cmd_values[2])

        self.cmd_pub.publish(cmd)
    
    def path_callback(self, msg):
        self.ref_path = [
            np.array([pose.pose.position.x, pose.pose.position.y], dtype=float)
            for pose in msg.poses
        ]

    def _read_control_horizon(self):
        """根据参数设置计算控制时域长度"""
        requested = int(self.get_parameter("control_horizon").value)
        if requested <= 0:
            fraction = float(
                self.get_parameter("control_horizon_fraction").value
            )
            requested = int(math.ceil(self.np_horizon * fraction))
        return max(1, min(self.np_horizon, requested))

    def scan_callback(self, msg):
        """激光雷达数据的回调函数,用于处理激光雷达数据并提取障碍物信息
        过滤掉无效数据，只筛选出距离小车最近、且彼此不扎堆的几处关键障碍物点,并转换到小车本体坐标系(Body Frame)下"""
        candidates = [] #创建一个空列表，用来临时存放通过第一轮筛选的障碍物候选点
        min_range = max(float(msg.range_min), 0.0) #确定雷达的最小有效探测距离。如果雷达本身的 range_min 负数（异常值），就强制重置为 0.0
        max_range = min(float(msg.range_max), self.obstacle_max_range) #确定最大关注距离。雷达能看很远（比如 10 米），但为了避障，我们可能只关心小车周围 self.obstacle_max_range（比如 2 米）内的危险。
        angle = float(msg.angle_min)  #初始化当前光束的弧度角，从激光雷达扫描的起始角度 msg.angle_min 开始。

        #遍历雷达光束，坐标转换与扇区过滤
        for distance in msg.ranges:    #激光雷达返回的是一个一维数组 ranges，里面按顺序填满了每根光束测量出来的距离值。
            if math.isfinite(distance) and min_range <= distance <= max_range:  #过滤掉无穷大 inf（没照到物体）或 NaN（噪声）的无效距离。
                body_angle = self._wrap_angle(angle + self.laser_yaw) #获得“当前这一根”激光光束在小车本体坐标系（Body Frame）下的绝对朝向角度，并将其限制在 [-pi, pi] 范围内。
                if abs(body_angle) <= self.obstacle_sector_half_angle:  #扇区过滤：只考虑那些在小车前方扇形区域内的点，扇形以小车朝向为中心线，左右各 self.obstacle_sector_half_angle（例如 60 度）为边界。
                    # 极坐标转直角坐标，并消除安装偏移
                    # 得到障碍物在小车本体坐标系（Base_link/Footprint）下的真实坐标。
                    x_body = (
                        self.laser_offset_x
                        + distance * math.cos(body_angle)
                    )
                    y_body = (
                        self.laser_offset_y
                        + distance * math.sin(body_angle)
                    )
                    candidates.append((distance, x_body, y_body))
            angle += float(msg.angle_increment) #步进。当前光束处理完，角度加上单步增量 angle_increment，准备处理下一根光束。

        candidates.sort(key=lambda point: point[0]) #按距离从小到大排序
        selected = []  #用来存放最终留下来的高质量障碍物点。
        for _, x_body, y_body in candidates:
            if all(              #all() 是 Python 的内置函数，all() 用于判断给定的可迭代对象（如列表、元组、生成器表达式等）中的所有元素是否都为 True。
                math.hypot(x_body - sx, y_body - sy) #计算当前候选点与已经选中的点（selected）之间的欧氏距离，因为想要障碍物点也离散一点，所以加上了一个最小分离距离 self.obstacle_min_separation（例如 0.20 米）的要求，只有当当前候选点与所有已选点的距离都大于等于这个值时，才会被加入到 selected 列表中。
                >= self.obstacle_min_separation
                for sx, sy in selected
            ):
                selected.append((x_body, y_body))
            if len(selected) >= self.max_obstacle_points:
                break

        self.lidar_obstacle_points_body = selected
        self.last_scan_time = self.get_clock().now() 
    def solve_nmpc_cbf(self):
        """求解NMPC+CBF问题的函数"""
        #如果没有当前状态或参考路径，则返回全0的控制命令,原地等待
        self.get_logger().debug("Entering solve_nmpc_cbf")
        if self.current_state is None or not self.ref_path:
            return np.zeros(3)
        #如果没有安装casadi，则使用一个简单的比例控制器作为回退方案
        if ca is None:
            if not self._reported_no_casadi:
                self.get_logger().warning(
                    "casadi is not installed; using proportional fallback"
                )
                self._reported_no_casadi = True
            cmd = self._clip_command(self._fallback_command())
            #np.tile 函数的作用是通过“平铺（复制）”指定的次数，来构造一个新的数组,(1, self.np_horizon)表示在列方向上复制self.np_horizon次，行方向上复制1次
            self.last_u_sequence = np.tile(
                cmd.reshape(3, 1),
                (1, self.np_horizon),
            )
            return cmd
        
        refs = self._reference_window()  #获取当前时刻起的预测时域长度的参考路径点
        obstacles = self._active_obstacles()  #获取当前时刻对NMPC问题有影响的障碍物信息
        warm_u = self._warm_start_sequence()
        warm_x = self._rollout_states(self.current_state, warm_u)

        opti = ca.Opti()
        x = opti.variable(3, self.np_horizon + 1)
        u = opti.variable(3, self.nc_horizon)
        cbf_slack = None
        if obstacles:
            cbf_slack = opti.variable(len(obstacles), self.np_horizon) #这是工业级算法的灵魂。如果障碍物突然贴脸，数学上可能出现“绝对撞墙、无解”的情况，
                                                                    #求解器会直接报错。引入一个大于 0 的松弛变量后，允许算法临时稍微“违规”一点点（作为代价），保证程序永远不崩溃，依然能算出逃逸的最优解
            opti.subject_to(ca.reshape(cbf_slack, -1, 1) >= 0)

        opti.subject_to(x[:, 0] == ca.DM(self.current_state))

        obj = 0
        q_xy = 12.0
        q_terminal = 20.0
        r_u = 0.08
        r_du = 0.4

        for k in range(self.nc_horizon):
            opti.subject_to(u[0, k] >= -self.max_vx)
            opti.subject_to(u[0, k] <= self.max_vx)
            opti.subject_to(-self.max_vy <= u[1, k])
            opti.subject_to(u[1, k] <= self.max_vy)
            opti.subject_to(-self.max_wz <= u[2, k])
            opti.subject_to(u[2, k] <= self.max_wz)

        for k in range(self.np_horizon):
            theta = x[2, k]
            u_k = u[:, min(k, self.nc_horizon - 1)]
            #这部分告诉求解器，未来每一个时刻的 $(x, y, yaw)$ 必须符合运动学连续性，不能瞬间瞬移。
            opti.subject_to(
                x[0, k + 1]
                == x[0, k]
                + (u_k[0] * ca.cos(theta) - u_k[1] * ca.sin(theta)) * self.dt
            )
            opti.subject_to(
                x[1, k + 1]
                == x[1, k]
                + (u_k[0] * ca.sin(theta) + u_k[1] * ca.cos(theta)) * self.dt
            )
            opti.subject_to(x[2, k + 1] == x[2, k] + u_k[2] * self.dt)

            # 嵌套循环：为每一个障碍物套上 CBF 安全外壳
            for obs_index, (obs_x, obs_y, safe_r) in enumerate(obstacles):
                h = (x[0, k] - obs_x) ** 2 + (x[1, k] - obs_y) ** 2 - safe_r**2
                h_next = (
                    (x[0, k + 1] - obs_x) ** 2
                    + (x[1, k + 1] - obs_y) ** 2
                    - safe_r**2
                )
                opti.subject_to(
                    h_next + cbf_slack[obs_index, k]
                    >= (1.0 - self.cbf_gamma) * h
                )
                obj += self.cbf_slack_weight * cbf_slack[obs_index, k] ** 2

            ref = ca.DM(refs[k])
            obj += q_xy * ca.sumsqr(x[0:2, k] - ref) # 1. 惩罚偏离参考路径（期望跟踪得准）
            obj += r_u * ca.sumsqr(u_k) # 2. 惩罚过大的速度输入（期望省电、平稳）
            if k > 0:    # 3. 惩罚速度剧烈抖动（期望加速度平滑，保护麦轮减速箱）
                u_prev = u[:, min(k - 1, self.nc_horizon - 1)]
                obj += r_du * ca.sumsqr(u_k - u_prev)

        #终端代价（终点要尽可能逼近目标）
        obj += q_terminal * ca.sumsqr(
            x[0:2, self.np_horizon] - ca.DM(refs[-1])
        )

        opti.minimize(obj)
        opti.set_initial(x, ca.DM(warm_x))
        opti.set_initial(u, ca.DM(warm_u[:, : self.nc_horizon]))
        if cbf_slack is not None:
            opti.set_initial(cbf_slack, 0.0)

        opts = {
            "print_time": 0,
            "ipopt.print_level": 0,
            "ipopt.max_iter": 300,
            "ipopt.sb": "yes",
            'ipopt.tol': 1e-3,              # 适当放宽容忍度（默认 1e-8 可能太严格）
            'ipopt.acceptable_tol': 1e-2,   # 可接受解的容忍度
        }
        opti.solver("ipopt", opts)

        try:
            solution = opti.solve()
            solved_u = np.array(solution.value(u), dtype=float)
            self.last_u_sequence = self._expand_control_sequence(solved_u)
            cmd = self.last_u_sequence[:, 0]
        except RuntimeError as exc:
            self.get_logger().warning(
                f"NMPC solve failed, using proportional fallback: {exc}",
                throttle_duration_sec=2.0,
            )
            cmd = self._clip_command(self._fallback_command())
            self.last_u_sequence = np.tile(
                cmd.reshape(3, 1),
                (1, self.np_horizon),
            )
        self.get_logger().debug("Exiting solve_nmpc_cbf")
        return self._clip_command(cmd)

    def _expand_control_sequence(self, solved_u):
        """将求解得到的控制时域长度的控制序列，扩展到预测时域长度，方便下一次求解时使用"""
        if solved_u.ndim == 1:
            solved_u = solved_u.reshape(3, 1)

        sequence = np.empty((3, self.np_horizon), dtype=float)
        for k in range(self.np_horizon):
            sequence[:, k] = solved_u[:, min(k, self.nc_horizon - 1)]
        return sequence
    
    def _warm_start_sequence(self):
        """生成一个用于NMPC求解的初始控制序列，基于上一次求解得到的控制序列进行错位平移，并在末尾填充最后一个控制输入"""
        if self.last_u_sequence is None: #如果是第一次运行，没有任何历史经验，那就用当前时刻的比例控制命令，平铺一整条序列。
            seed = self._clip_command(self._fallback_command())
            sequence = np.tile(seed.reshape(3, 1), (1, self.np_horizon))
        else:
            sequence = np.empty_like(self.last_u_sequence)
            sequence[:, :-1] = self.last_u_sequence[:, 1:] # 错开一位，把历史的第 2 步移到第 1 步
            sequence[:, -1] = self.last_u_sequence[:, -1] # 最后一步用最后的值复制填充

        if self.nc_horizon < self.np_horizon:
            sequence[:, self.nc_horizon :] = sequence[
                :,
                self.nc_horizon - 1 : self.nc_horizon,
            ]
        return sequence
    
    def _rollout_states(self, state, controls):
        """基于当前状态和一个控制序列，按照小车的运动学模型，前向滚动预测得到一个状态序列,用于初始化NMPC求解器中的状态变量"""
        states = np.empty((3, self.np_horizon + 1), dtype=float)
        states[:, 0] = state
        for k in range(self.np_horizon):
            x, y, yaw = states[:, k]
            vx, vy, wz = controls[:, k]
            states[0, k + 1] = x + (
                vx * math.cos(yaw) - vy * math.sin(yaw)
            ) * self.dt
            states[1, k + 1] = y + (
                vx * math.sin(yaw) + vy * math.cos(yaw)
            ) * self.dt
            states[2, k + 1] = self._wrap_angle(yaw + wz * self.dt)
        return states
    
    def _fallback_command(self):
        """当没有CasADi时的回退控制命令计算
            比例控制P控制+ 人工势场法APF
        """
        if self.current_state is None or not self.ref_path:
            return np.zeros(3)

        refs = self._reference_window()
        target = refs[min(3, len(refs) - 1)] #从未来的参考路径窗口中，挑选出往前数第 3 个点作为当前的临时追赶目标（看远一点，小车行驶更平滑）
        # 计算小车当前位置到目标点在世界坐标系下的 $X, Y$ 轴距离偏差。
        dx = target[0] - self.current_state[0]
        dy = target[1] - self.current_state[1]
        yaw = self.current_state[2]
        #把世界坐标系下的偏差 (dx, dy) 转换到小车自身底盘坐标系（Body Frame）下的前后偏差 body_x 和左右偏差 body_y。
        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cmd = np.array([1.2 * body_x, 1.2 * body_y, 0.0])

        #人工势场法 APF 简化版
        """
        只奔向目标是不行的，如果路上有墙，小车会一头撞上去。于是代码在这里套用了一个
        极其经典的人工势场法Artificial Potential Field思想:
        把障碍物当成磁铁的同极，越靠近，排斥力越强
        """
        for obs_x, obs_y in self.lidar_obstacle_points_body:
            distance = max(math.hypot(obs_x, obs_y), 1.0e-3)#直角三角形斜边公式,算出了这个障碍物距离小车中心点的真实直线距离，并且加了一个下限，防止除零错误
            limit = self.robot_radius + self.safety_margin + 0.4 #这是小车开始感到危险的距离阈值（警戒线半径）
            if distance < limit:
                strength = 0.6 * (limit - distance) / limit #归一化反比例公式，用来控制避障的“急迫程度”
                """
                obs_x / distance把具体的坐标转化为了单位向量。它代表了从“小车中心”指向“障碍物”的方向。
                既然 obs_x / distance 是从小车指向障碍物的方向，那么前面加个负号（减法），
                就变成了从障碍物把小车推开的反方向！
                """
                cmd[0] -= strength * obs_x / distance
                cmd[1] -= strength * obs_y / distance

        return cmd

    def _reference_window(self):
        """获取当前时刻起的预测时域长度的参考路径点"""
        points = np.vstack(self.ref_path)
        distances = np.linalg.norm(points - self.current_state[:2], axis=1)
        start_index = int(np.argmin(distances))
        refs = []
        for k in range(self.np_horizon + 1):
            refs.append(points[min(start_index + k, len(points) - 1)])#当小车快开到终点时，后面的点不够数了，这个 min 函数会强行把索引
        return refs                                                   #卡在最后一个点。这意味着如果快到终点了，未到步长的后续参考点将全部由终点坐标填充
    def _active_obstacles(self):        
        """获取当前时刻对NMPC问题有影响的障碍物信息,充当一个分支调度器,根据激光雷达数据是否存在或者是否过期，来决定返回基于激光雷达提取的动态障碍物点，还是返回预设的静态障碍物信息"""
        if self.use_lidar_obstacles:
            if not self._scan_is_fresh(): #调用上面的新鲜度检查。如果雷达断开了，为了防止 NMPC 使用过期的旧数据做出错误决策，这里果断返回一个空列表 []（即本次循环不考虑雷达障碍物）。这也是一种控制层面的“降级”保护。
                return []
            return self._lidar_points_to_world_obstacles() #如果雷达数据很新鲜，则调用另一个转换函数，把刚才回调函数里筛选出的小车本体坐标系下的雷达点，转换到世界/里程计坐标系（Map/Odom）下，供 NMPC 统一求解。
        
        #未启用雷达，使用地图预设的静态障碍物（并进行安全膨胀）
        safe_margin = self.robot_radius + self.safety_margin
        return [
            (float(obs_x), float(obs_y), safe_margin + float(obs_r))
            for obs_x, obs_y, obs_r in self.static_obstacles
        ]
    def _scan_is_fresh(self):
        """判断激光雷达数据是否新鲜，即是否在设定的超时时间内"""
        if self.last_scan_time is None: #检查是否从来没有收到过任何雷达数据（我们在sacn_callback回调函数末尾会更新这个时间）。
            self.get_logger().warning(
                "No LaserScan received yet; NMPC is running without "
                "lidar CBF constraints",
                throttle_duration_sec=2.0, #限流打印，每 2 秒最多打印一次，防止频繁刷屏。
            )
            return False

        #计算数据的年龄（延迟）。用当前系统时间减去最后一次收到雷达数据的时间，将纳秒（nanoseconds）乘以 $10^{-9}$ 转换为秒
        age = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds * 1.0e-9
        if age > self.scan_timeout:  #超时判定。如果数据的年龄超过了设定的阈值，说明雷达话题卡死了（Stale/老旧）
            self.get_logger().warning(
                "LaserScan is stale; NMPC is running without lidar "
                "CBF constraints",
                throttle_duration_sec=2.0,
            )
            return False
        return True
    
    def _read_obstacles(self):
        values = list(self.get_parameter("obstacles").value)
        if len(values) % 3 != 0:
            self.get_logger().warning(
                "obstacles must be flat triples: [x, y, radius, ...]"
            )
            return []
        return [values[i : i + 3] for i in range(0, len(values), 3)]
    
    def _lidar_points_to_world_obstacles(self):
        """将小车本体坐标系下的雷达障碍物点转换到世界/里程计坐标系下
            利用当前小车在世界坐标系下的位置和朝向（位姿），将小车身体周围的雷达障碍物点，
            “投影”到世界地图中，并完成安全膨胀。
        """
        if self.current_state is None:  #安全检查。如果小车刚启动，还没有收到里程计（Odom）发布的自身位置数据，此时无法进行空间转换，果断返回空列表 []
            return []

        x, y, yaw = self.current_state
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        safe_distance = self.robot_radius + self.safety_margin
        obstacles = []
        for px_body, py_body in self.lidar_obstacle_points_body:
            px_world = x + cos_yaw * px_body - sin_yaw * py_body
            py_world = y + sin_yaw * px_body + cos_yaw * py_body
            obstacles.append((px_world, py_world, safe_distance))
        return obstacles
    
    def _clip_command(self, cmd):
        return np.array(
            [
                np.clip(cmd[0], -self.max_vx, self.max_vx),
                np.clip(cmd[1], -self.max_vy, self.max_vy),
                np.clip(cmd[2], -self.max_wz, self.max_wz),
            ],
            dtype=float,
        )

    def imu_callback(self, msg):
        if not self.use_imu_yaw: #检查你的控制节点是否开启了 IMU 融合功能（这是一个布尔值参数）。
            return             #如果用户在参数服务器或者 Launch 文件里关闭了 IMU（比如只想纯用轮式里程计），
                                #那么这个回调函数在第一行就会直接跳出，后面所有的数学计算都不会执行，
                                # 以此来节省小车的处理器算力。

        q = msg.orientation  #从 ROS 2 的标准惯导消息 sensor_msgs/msg/Imu 中，提取出表示当前小车 3D 朝向的四元数结构体 q（包含 x, y, z, w 四个分量）
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1.0e-6:  #这里设定如果模长接近于 0（小于 $10^{-6}$），判定为非法垃圾数据，果断直接丢弃。
            return   

        self.latest_imu_yaw = self._yaw_from_quaternion(q)
        self.latest_imu_time = self.get_clock().now()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position #从 ROS 2 的标准里程计消息 nav_msgs/msg/Odometry 中，提取出小车的 3D 坐标 $(X, Y, Z)$。对于平面麦轮小车，我们待会儿只拿其中的 x 和 y。
        odom_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        yaw = self._fused_yaw(odom_yaw)
        self.current_state = np.array([pos.x, pos.y, yaw], dtype=float)
    
    def _fused_yaw(self, odom_yaw):
        """麦克纳姆轮由于极易打滑，导致轮子编码器算出来的里程计朝向（odom_yaw）时间长了会严重漂移。
        而 IMU 的陀螺仪短时间内对角度的变化非常敏感。这个函数就是要把两者的长处结合起来"""
        #如果你关闭了 IMU 融合开关（use_imu_yaw 为 False），或者还没收到任何 IMU 数据，则直接信任并返回里程计的角度。
        if not self.use_imu_yaw or self.latest_imu_yaw is None:
            return odom_yaw
        if self.latest_imu_time is None:
            return odom_yaw

        age = (
            self.get_clock().now() - self.latest_imu_time
        ).nanoseconds * 1.0e-9
        if age > self.imu_timeout:   #计算当前时间与上一帧 IMU 时间的差值（秒）。如果 IMU 超时了（比如断线超过 0.2 秒），立刻放弃融合，直接返回 odom_yaw，确保控制系统不会因为使用了错误的过时传感器数据而跑飞。
            return odom_yaw
        #动态零位对齐（Offset 校准）
        """
        IMU 刚开机时的 0 circ比如正北方和里程计开机时的 0 circ小车车头朝向通常是不一致的。
        当系统第一次运行到这里时，它会计算两者之间的夹角差值，并存进 self.imu_yaw_offset。
        以后所有的 IMU 数据都要补上这个偏置。
        """
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = self._wrap_angle(
                odom_yaw - self.latest_imu_yaw
            )

        imu_yaw = self._wrap_angle(self.latest_imu_yaw + self.imu_yaw_offset)
        return self._blend_angles(odom_yaw, imu_yaw, self.imu_yaw_weight)

    @staticmethod   #角度归一化函数，将任意角度转换到 -pi 到 pi 的范围内
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod   #经典的四元数转航向角的数学公式
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod #角度融合函数，基于权重对两个角度进行平均，注意这里的平均是基于单位圆上的向量平均，而不是简单的数值平均，以避免角度环绕问题
    def _blend_angles(angle_a, angle_b, weight_b):
        weight_b = float(np.clip(weight_b, 0.0, 1.0))  #使用 np.clip 确保 IMU 的权重 weight_b 严格限制在 [0.0, 1.0] 之间（防止调参时不小心输入了 1.2 这样的非法值导致系统发散）
        weight_a = 1.0 - weight_b
        #映射到单位圆（分解为 X, Y 分量）
        sin_sum = weight_a * math.sin(angle_a) + weight_b * math.sin(angle_b)
        cos_sum = weight_a * math.cos(angle_a) + weight_b * math.cos(angle_b)
        return math.atan2(sin_sum, cos_sum)  #这个合成后的向量重新翻译回 [-pi, pi] 之间的标准弧度角。
    
def main():
    rclpy.init()
    node = nmpc_cbf_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 1. 在这里创建一个全 0 的紧急刹车消息
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0

        # 2. 强行发布出去
        node.cmd_pub.publish(stop_cmd)

        # 3. 释放节点资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()