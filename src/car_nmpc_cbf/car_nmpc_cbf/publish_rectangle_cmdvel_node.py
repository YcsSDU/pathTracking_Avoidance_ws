#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class MecanumRectangleDriveNode(Node):
    def __init__(self):
        super().__init__('mecanum_rectangle_drive_node')
        
        # 1. 初始化发布者 (使用 TwistStamped)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        # 2. 运动参数配置
        # 提示：麦克纳姆轮侧向滑行时由于辊子摩擦，实际速度可能略低于前后直行，可单独微调
        self.speed_x = 0.2    # 前后纵向速度 (m/s)
        self.speed_y = 0.2    # 左右横向速度 (m/s)
        
        # 麦克纳姆轮平移矩形的 4 个阶段持续时间 (秒)
        self.state_durations = {
            'MOVE_FORWARD':  4.0,   # 阶段 1：向前平移（长边）
            'MOVE_LEFT':     2.0,   # 阶段 2：向左平移（短边）
            'MOVE_BACKWARD': 4.0,   # 阶段 3：向后平移（长边）
            'MOVE_RIGHT':    2.0    # 阶段 4：向右平移（短边）
        }
        
        # 3. 状态机初始化
        self.states = ['MOVE_FORWARD', 'MOVE_LEFT', 'MOVE_BACKWARD', 'MOVE_RIGHT']
        self.current_state_idx = 0
        self.state_start_time = self.get_clock().now()
        
        # 4. 高频控制定时器 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("🚀 麦克纳姆轮全向矩形循环节点已启动（车头方向保持固定）！")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds / 1e9
        
        current_state = self.states[self.current_state_idx]
        target_duration = self.state_durations[current_state]
        
        # 状态切换逻辑
        if elapsed_time >= target_duration:
            self.current_state_idx = (self.current_state_idx + 1) % len(self.states)
            self.state_start_time = now
            current_state = self.states[self.current_state_idx]
            self.get_logger().info(f"🔄 切换麦轮运动状态至: {current_state}")

        # 构建 TwistStamped 消息
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'

        # 麦克纳姆轮全向速度解算映射
        # 注意：这里完全不需要分配角速度 angular.z，全程靠 linear.x 和 linear.y 的组合
        if current_state == 'MOVE_FORWARD':
            msg.twist.linear.x = self.speed_x
            msg.twist.linear.y = 0.0
        elif current_state == 'MOVE_LEFT':
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = self.speed_y    # ROS 2 坐标系中，Y正方向为左
        elif current_state == 'MOVE_BACKWARD':
            msg.twist.linear.x = -self.speed_x   # X负方向为后
            msg.twist.linear.y = 0.0
        elif current_state == 'MOVE_RIGHT':
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = -self.speed_y   # Y负方向为右

        # 保持自转角速度为0
        msg.twist.angular.z = 0.0

        # 发布速度
        self.cmd_pub.publish(msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = MecanumRectangleDriveNode()
    
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