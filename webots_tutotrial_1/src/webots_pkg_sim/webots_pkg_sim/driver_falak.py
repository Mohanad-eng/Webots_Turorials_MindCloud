import rclpy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class FalakDriver:

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._timestep = int(self._robot.getBasicTimeStep())

        self._linear_x = 0.0
        self._angular_z = 0.0

        # ── Robot Geometry (from URDF) ─────────────────────
        self.WHEEL_RADIUS = 0.086
        self.HALF_TRACK_WIDTH = 0.2862   # y-distance from center to wheel

        self.LEFT_WHEELS = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint']
        self.RIGHT_WHEELS = ['wheel4_joint', 'wheel5_joint', 'wheel6_joint']
        self.ALL_WHEELS = self.LEFT_WHEELS + self.RIGHT_WHEELS

        # ── ROS2 Node ─────────────────────────────────────
        rclpy.init(args=None)
        self._node = rclpy.create_node("falak_driver")

        # ── Motors ────────────────────────────────────────
        self._wheels = {}
        for name in self.ALL_WHEELS:
            motor = self._robot.getDevice(name)
            if motor is None:
                self._node.get_logger().error(f"Motor not found: {name}")
                continue
            motor.setPosition(float('inf'))   # velocity control mode
            motor.setVelocity(0.0)
            self._wheels[name] = motor

        # ── Subscribers ───────────────────────────────────
        self._node.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )

        # ── Publishers ────────────────────────────────────
        self._joint_pub = self._node.create_publisher(
            JointState, 'joint_states', 10
        )

        self._node.get_logger().info("Falak driver initialized successfully")

    # ───────────────────────────────────────────────────────
    def _cmd_vel_callback(self, msg: Twist):
        self._linear_x = msg.linear.x
        self._angular_z = msg.angular.z
        print(self._linear_x)

    # ───────────────────────────────────────────────────────
    def _compute_wheel_velocities(self):
        """Standard differential drive inverse kinematics"""
        vx = self._linear_x
        wz = self._angular_z

        # Linear velocity at left and right sides
        v_left = vx - wz * self.HALF_TRACK_WIDTH
        v_right = vx + wz * self.HALF_TRACK_WIDTH

        # Convert to angular velocity (rad/s)
        omega_left = v_left / self.WHEEL_RADIUS
        omega_right = v_right / self.WHEEL_RADIUS

        return omega_left, omega_right

    # ───────────────────────────────────────────────────────
    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

        # Compute wheel speeds
        omega_left, omega_right = self._compute_wheel_velocities()

        # Apply to motors
        for name in self.LEFT_WHEELS:
            if name in self._wheels:
                self._wheels[name].setVelocity(omega_left)

        for name in self.RIGHT_WHEELS:
            if name in self._wheels:
                self._wheels[name].setVelocity(omega_right)

        # ── Publish Joint States ───────────────────────────
        stamp = self._node.get_clock().now().to_msg()

        js = JointState()
        js.header = Header()
        js.header.stamp = stamp
        js.name = self.ALL_WHEELS

        # All left wheels have same velocity, all right have same
        js.velocity = [omega_right] * 3 + [omega_left] * 3
        js.position = [0.0] * 6   # We don't track position here

        self._joint_pub.publish(js)

        return self._robot.step(self._timestep)