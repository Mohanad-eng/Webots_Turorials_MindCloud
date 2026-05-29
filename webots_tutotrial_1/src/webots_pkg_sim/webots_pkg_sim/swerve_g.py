#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np


class SwerveController(Node):

    def __init__(self):
        super().__init__('swerve_controller')

        self.get_logger().info("✅ Swerve Controller Started")

        # Robot geometry (adjust if needed)
        self.L = 0.7   # length (front-back distance)
        self.W = 0.6   # width  (left-right distance)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Controller mapping (MATCHES YOUR URDF)
        self.modules = {
            'fl': ('steering1_controller', 'wheel1_controller'),
            'fr': ('steering2_controller', 'wheel2_controller'),
            'rl': ('steering3_controller', 'wheel3_controller'),
            'rr': ('steering4_controller', 'wheel4_controller'),
        }

        # Publishers
        self.steer_pubs = {}
        self.drive_pubs = {}

        for key, (steer, drive) in self.modules.items():
            self.steer_pubs[key] = self.create_publisher(
                Float64, f'/{steer}/command', 10)

            self.drive_pubs[key] = self.create_publisher(
                Float64, f'/{drive}/command', 10)

        # Store previous angles (for optimization)
        self.prev_angles = {k: 0.0 for k in self.modules}

    # ─────────────────────────────────────────────
    def cmd_callback(self, msg):

        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z

        # --- Swerve kinematics ---
        R = np.sqrt(self.L**2 + self.W**2)

        A = vx - w * (self.L / R)
        B = vx + w * (self.L / R)
        C = vy - w * (self.W / R)
        D = vy + w * (self.W / R)

        speeds = {
            'fl': np.sqrt(B**2 + D**2),
            'fr': np.sqrt(B**2 + C**2),
            'rl': np.sqrt(A**2 + D**2),
            'rr': np.sqrt(A**2 + C**2)
        }

        angles = {
            'fl': np.arctan2(D, B),
            'fr': np.arctan2(C, B),
            'rl': np.arctan2(D, A),
            'rr': np.arctan2(C, A)
        }

        # --- Normalize speeds ---
        max_speed = max(speeds.values())
        if max_speed > 1.0:
            for k in speeds:
                speeds[k] /= max_speed

        # --- Steering optimization ---
        for k in self.modules:

            target = angles[k]
            current = self.prev_angles[k]

            diff = self.angle_diff(target, current)

            # If rotation > 90°, flip wheel direction
            if abs(diff) > np.pi / 2:
                target = self.wrap_angle(target + np.pi)
                speeds[k] *= -1

            self.prev_angles[k] = target

            # --- Publish steering ---
            steer_msg = Float64()
            steer_msg.data = float(target)

            # --- Publish wheel speed ---
            drive_msg = Float64()
            drive_msg.data = float(speeds[k])

            self.steer_pubs[k].publish(steer_msg)
            self.drive_pubs[k].publish(drive_msg)

    # ─────────────────────────────────────────────
    def angle_diff(self, a, b):
        return self.wrap_angle(a - b)

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main():
    rclpy.init()
    node = SwerveController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()