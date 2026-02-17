#!/usr/bin/env python3

import math
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class DummyPlanner(Node):

    def __init__(self):
        super().__init__('dummy_planner')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            qos
        )
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )

        self.current_pose: Optional[PoseStamped] = None
        self.goal_pose: Optional[PoseStamped] = None

        self.get_logger().info("Dummy Planner Initialized")

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def goal_callback(self, msg: PointStamped):
        self.get_logger().info("Received clicked point")

        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point

        # Default orientation (face forward)
        goal_pose.pose.orientation.w = 1.0

        self.goal_pose = goal_pose

        self.plan_and_publish()

    def plan_and_publish(self):

        if self.current_pose is None:
            self.get_logger().warn("No odom yet — cannot plan")
            return

        if self.goal_pose is None:
            return

        start = self.current_pose.pose.position
        goal = self.goal_pose.pose.position

        path_points = self.generate_straight_line(
            (start.x, start.y),
            (goal.x, goal.y),
            resolution=0.05
        )

        path_msg = Path()
        path_msg.header.frame_id = self.goal_pose.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        yaw = math.atan2(goal.y - start.y, goal.x - start.x)

        for (x, y) in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orientation toward goal
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            pose.pose.orientation.z = math.sin(yaw / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses")

    def generate_straight_line(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        resolution: float = 0.05
    ) -> List[Tuple[float, float]]:

        sx, sy = start
        gx, gy = goal

        distance = math.hypot(gx - sx, gy - sy)
        steps = max(int(distance / resolution), 2)

        points = []

        for i in range(steps + 1):
            t = i / steps
            x = sx + t * (gx - sx)
            y = sy + t * (gy - sy)
            points.append((x, y))

        return points


def main(args=None):
    rclpy.init(args=args)
    node = DummyPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
