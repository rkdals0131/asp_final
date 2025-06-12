#!/usr/bin/env python3
#차량제어

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import os
import csv
import transforms3d.euler


class PurePursuitMultiWaypoint(Node):
    def __init__(self):
        super().__init__('pure_pursuit_multi_waypoint')

        self.publisher = self.create_publisher(Twist, '/model/X1_asp/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/model/X1/odometry', self.odom_callback, 10)

        self.lookahead_distance = 2.0
        self.linear_velocity = 2.0
        self.threshold = 3.0  # 도달 판정 거리

        self.current_pos = (0.0, 0.0)
        self.current_yaw = 0.0

        self.waypoints = self.load_waypoints()
        self.current_idx = 0
        self.arrived = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'{len(self.waypoints)}개 웨이포인트 로딩 완료.')

    def load_waypoints(self):
        waypoints = [
            (31.19, -0.54),
            (31.83, -6.82),
            (27.65, -16.80),
            (27.18, -19.44),
            (23.63, -19.80),
            (19.17, -29.87),
            (18.98, -22.74),
            (15.25, -30.66),
            (13.26, -39.43),
            (7.26, -46.02),
            (2.27, -49.41),
            (-2.75, -52.08),
            (-9.16, -60.97),
        ]
        return waypoints

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_pos = (pos.x, pos.y)
        self.current_yaw = transforms3d.euler.quat2euler([ori.w, ori.x, ori.y, ori.z])[2]

    def timer_callback(self):
        if self.arrived or self.current_idx >= len(self.waypoints):
            self.stop_robot()
            return

        tx, ty = self.waypoints[self.current_idx]
        x, y = self.current_pos
        dist = math.hypot(tx - x, ty - y)

        if dist < self.threshold:
            self.get_logger().info(f'✅ [{self.current_idx}] 웨이포인트 도달: ({tx:.2f}, {ty:.2f})')
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.get_logger().info('🏁 모든 웨이포인트 도달. 정지합니다.')
                self.arrived = True
            return

        dx = tx - x
        dy = ty - y

        target_yaw = math.atan2(dy, dx)
        alpha = target_yaw - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap

        Ld = math.hypot(dx, dy)
        curvature = 2 * math.sin(alpha) / Ld if Ld != 0 else 0.0
        angular_z = self.linear_velocity * curvature

        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = angular_z
        self.publisher.publish(msg)

        self.get_logger().info(f'🎯 {self.current_idx}번 목표점: ({tx:.2f}, {ty:.2f}) | 현재: ({x:.2f}, {y:.2f}) | 거리: {dist:.2f}m | α={math.degrees(alpha):.2f}° | ω={angular_z:.2f}')

    def stop_robot(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitMultiWaypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
