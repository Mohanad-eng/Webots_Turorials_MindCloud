"""
falak_driver.py  -  Webots ROS2 extern controller for the falak rover.

Architecture: 6-wheel skid-steer (differential drive).
  Right side wheels: wheel1 (rear), wheel2 (mid), wheel3 (front) -> axis 0 -1 0
  Left  side wheels: wheel4 (front), wheel5 (mid), wheel6 (rear) -> axis 0  1 0

  Because all wheels on the same side share the same axis sign, a single
  velocity command drives all three wheels on that side identically.

Kinematics (cmd_vel -> wheel velocities):
  v_right =  (lin_x - ang_z * HALF_TRACK) / WHEEL_RADIUS
  v_left  =  (lin_x + ang_z * HALF_TRACK) / WHEEL_RADIUS

  Right wheels: send v_right  (axis 0 -1 0 -> positive = forward)
  Left  wheels: send v_left   (axis 0  1 0 -> positive = forward)

Published topics:
  /joint_states          (sensor_msgs/JointState)
  /imu                   (sensor_msgs/Imu)
  /gps/fix               (sensor_msgs/NavSatFix)
  /camera/image_raw      (sensor_msgs/Image)
  /camera/depth/image_raw(sensor_msgs/Image)
  /camera/camera_info    (sensor_msgs/CameraInfo)
  /camera/depth/camera_info (sensor_msgs/CameraInfo)

Subscribed topics:
  /cmd_vel               (geometry_msgs/TwistStamped)
"""

import math
import struct

import rclpy
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import (
    CameraInfo, Image, Imu, JointState, NavSatFix
)
from tf2_ros import StaticTransformBroadcaster

# -- Robot geometry ------------------------------------------------------------
WHEEL_RADIUS = 0.1141   # metres  (derived from URDF inertia: R=sqrt(2*iyy/m))
HALF_TRACK   = 0.2862   # half of track width (Y offset of wheel centres)

# Camera field-of-view (radians) - matches .wbt RangeFinder/Camera
CAMERA_FOV = 1.0472   # ~= 60deg

# Wheel joint names in the order they will appear in JointState.name
RIGHT_WHEELS = ["wheel1_joint", "wheel2_joint", "wheel3_joint"]
LEFT_WHEELS  = ["wheel4_joint", "wheel5_joint", "wheel6_joint"]
ALL_WHEELS   = RIGHT_WHEELS + LEFT_WHEELS


class FalakDriver:
    """Webots plugin class (called by webots_ros2_driver)."""

    # -------------------------------------------------------------------------
    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._lin_x = 0.0
        self._ang_z = 0.0
        self._step_count = 0
        self._depth_ready = False

        rclpy.init(args=None)
        self._node = rclpy.create_node("falak_driver")

        timestep = int(self._robot.getBasicTimeStep())

        # -- Wheel motors ------------------------------------------------------
        self._motors = {}
        for name in ALL_WHEELS:
            m = self._robot.getDevice(name)
            if m is None:
                self._node.get_logger().error(
                    f"Motor '{name}' not found in Webots robot!")
                continue
            m.setPosition(float("inf"))  # velocity-control mode
            m.setVelocity(0.0)
            self._motors[name] = m

        # -- Wheel position sensors --------------------------------------------
        self._sensors = {}
        for name in ALL_WHEELS:
            sensor_name = name.replace("_joint", "_sensor")
            s = self._robot.getDevice(sensor_name)
            if s:
                s.enable(timestep)
                self._sensors[name] = s

        # -- IMU / Accelerometer / Gyro ----------------------------------------
        self._imu   = self._robot.getDevice("imu")
        self._accel = self._robot.getDevice("accelerometer")
        self._gyro  = self._robot.getDevice("gyro")
        for dev in (self._imu, self._accel, self._gyro):
            if dev:
                dev.enable(timestep)

        # -- GPS ---------------------------------------------------------------
        self._gps = self._robot.getDevice("gps")
        if self._gps:
            self._gps.enable(timestep)

        # -- RGB camera --------------------------------------------------------
        self._camera = self._robot.getDevice("oakd_rgb")
        if self._camera:
            self._camera.enable(timestep)

        # -- Depth camera ------------------------------------------------------
        self._depth = self._robot.getDevice("oakd_depth")
        if self._depth:
            try:
                self._depth.enable(timestep)
            except Exception as e:
                self._node.get_logger().error(
                    f"Depth sensor enable failed: {e}")
                self._depth = None

        # -- ROS subscriber ----------------------------------------------------
        self._node.create_subscription(
            TwistStamped, "cmd_vel", self._cmd_vel_cb, 1)

        # -- ROS publishers ----------------------------------------------------
        self._pub_joints     = self._node.create_publisher(
            JointState,  "joint_states",              1)
        self._pub_imu        = self._node.create_publisher(
            Imu,         "imu",                       10)
        self._pub_gps        = self._node.create_publisher(
            NavSatFix,   "gps/fix",                   10)
        self._pub_rgb        = self._node.create_publisher(
            Image,       "camera/image_raw",           10)
        self._pub_depth      = self._node.create_publisher(
            Image,       "camera/depth/image_raw",     10)
        self._pub_cam_info   = self._node.create_publisher(
            CameraInfo,  "camera/camera_info",         10)
        self._pub_depth_info = self._node.create_publisher(
            CameraInfo,  "camera/depth/camera_info",   10)

        # -- Static TF ---------------------------------------------------------
        self._tf_broadcaster = StaticTransformBroadcaster(self._node)
        self._publish_static_transforms()

        self._node.get_logger().info("FalakDriver ready (6-wheel skid-steer)")

    def _cmd_vel_cb(self, msg: TwistStamped):
        self._lin_x = msg.twist.linear.x
        self._ang_z = msg.twist.angular.z
        self._node.get_logger().info(
            f"cmd_vel: lin_x={self._lin_x:.3f}  ang_z={self._ang_z:.3f}")

    def _skid_steer(self):
        MAX_VEL = 10.0

        # Physical wheel angular velocities
        v_r = (
            self._lin_x - self._ang_z * HALF_TRACK
        ) / WHEEL_RADIUS

        v_l = (
            self._lin_x + self._ang_z * HALF_TRACK
        ) / WHEEL_RADIUS

        # Convert to Webots motor signs
        right_cmd = -v_r   # because axis = (0,-1,0)
        left_cmd  =  v_l   # because axis = (0,+1,0)

        scale = max(abs(right_cmd), abs(left_cmd), MAX_VEL) / MAX_VEL
        right_cmd /= scale
        left_cmd /= scale

        return right_cmd, left_cmd
        # -------------------------------------------------------------------------
    def _publish_static_transforms(self):
        """Publish fixed TF transforms that never change."""
        transforms = []
        stamp = self._node.get_clock().now().to_msg()

        def _tf(parent, child, tx=0.0, ty=0.0, tz=0.0,
                qx=0.0, qy=0.0, qz=0.0, qw=1.0):
            t = TransformStamped()
            t.header.stamp         = stamp
            t.header.frame_id      = parent
            t.child_frame_id       = child
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x    = qx
            t.transform.rotation.y    = qy
            t.transform.rotation.z    = qz
            t.transform.rotation.w    = qw
            return t

        # Optical-frame quaternion: -90deg Z then -90deg X
        #   (converts ROS body frame to camera optical convention)
        OQ = dict(qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)

        transforms += [
            # base_link -> imu_link
            _tf("base_link",  "imu_link",              tz=0.10),
            # base_link -> gps_link
            _tf("base_link",  "gps_link",              tz=0.22),
            # base_link -> oakd_link  (camera body)
            _tf("base_link",  "oakd_link",  tx=0.45,   tz=0.15),
            # oakd_link -> optical frames
            _tf("oakd_link",  "oakd_rgb_optical_link",   **OQ),
            _tf("oakd_link",  "oakd_depth_optical_link", **OQ),
            # Webots names the RangeFinder frame after the device name
            _tf("oakd_link",  "oakd_depth",              **OQ),
        ]

        self._tf_broadcaster.sendTransform(transforms)
        self._node.get_logger().info("Static TF transforms published")

    # -------------------------------------------------------------------------
    def _make_camera_info(self, stamp, frame_id, w, h):
        """Build a CameraInfo message from image dimensions and FOV."""
        fx = w / (2.0 * math.tan(CAMERA_FOV / 2.0))
        fy = fx
        cx = w / 2.0
        cy = h / 2.0

        msg = CameraInfo()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.width  = w
        msg.height = h
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx,  0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    # -------------------------------------------------------------------------
    def step(self):
        #elf._node.get_logger().info("Static TF transforms published")
        self._step_count += 1
        if self._step_count < 2:
            return

        rclpy.spin_once(self._node, timeout_sec=0)
        stamp = self._node.get_clock().now().to_msg()

        # -- Drive -------------------------------------------------------------
        v_right, v_left = self._skid_steer()

        # DEBUG: print what we're actually sending
        self._node.get_logger().info(
            f"DRIVE -> v_right={v_right:.3f}  v_left={v_left:.3f}  "
            f"lin_x={self._lin_x:.3f}  ang_z={self._ang_z:.3f}",
            throttle_duration_sec=1)

        for name in RIGHT_WHEELS:
            if name in self._motors:
                self._motors[name].setVelocity(v_right)

        for name in LEFT_WHEELS:
            if name in self._motors:
                self._motors[name].setVelocity(v_left)
        # -- Joint states ------------------------------------------------------
        js = JointState()
        js.header.stamp = stamp
        js.name         = ALL_WHEELS
        js.position     = []
        js.velocity     = []
        for name in ALL_WHEELS:
            if name in self._sensors:
                js.position.append(float(self._sensors[name].getValue()))
            else:
                js.position.append(0.0)
            if name in self._motors:
                js.velocity.append(float(self._motors[name].getVelocity()))
            else:
                js.velocity.append(0.0)
        js.effort = [0.0] * len(ALL_WHEELS)
        self._pub_joints.publish(js)

        # -- IMU ---------------------------------------------------------------
        if self._imu and self._gyro and self._accel:
            try:
                rpy   = self._imu.getRollPitchYaw()
                gyro  = self._gyro.getValues()
                accel = self._accel.getValues()

                r_, p_, y_ = rpy
                cr, sr = math.cos(r_ / 2), math.sin(r_ / 2)
                cp, sp = math.cos(p_ / 2), math.sin(p_ / 2)
                cy, sy = math.cos(y_ / 2), math.sin(y_ / 2)

                imu_msg = Imu()
                imu_msg.header.stamp    = stamp
                imu_msg.header.frame_id = "imu_link"
                imu_msg.orientation.w   = cr * cp * cy + sr * sp * sy
                imu_msg.orientation.x   = sr * cp * cy - cr * sp * sy
                imu_msg.orientation.y   = cr * sp * cy + sr * cp * sy
                imu_msg.orientation.z   = cr * cp * sy - sr * sp * cy
                imu_msg.angular_velocity.x    = float(gyro[0])
                imu_msg.angular_velocity.y    = float(gyro[1])
                imu_msg.angular_velocity.z    = float(gyro[2])
                imu_msg.linear_acceleration.x = float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])
                imu_msg.orientation_covariance = [
                    0.01, 0.0, 0.0,
                    0.0,  0.01, 0.0,
                    0.0,  0.0,  0.01]
                imu_msg.angular_velocity_covariance = [
                    4e-8, 0.0,  0.0,
                    0.0,  4e-8, 0.0,
                    0.0,  0.0,  4e-8]
                imu_msg.linear_acceleration_covariance = [
                    3e-4, 0.0,  0.0,
                    0.0,  3e-4, 0.0,
                    0.0,  0.0,  3e-4]
                self._pub_imu.publish(imu_msg)
            except Exception as e:
                self._node.get_logger().warn(
                    f"IMU error: {e}", throttle_duration_sec=5)

        # -- GPS ---------------------------------------------------------------
        if self._gps:
            try:
                vals = self._gps.getValues()
                gps_msg = NavSatFix()
                gps_msg.header.stamp    = stamp
                gps_msg.header.frame_id = "gps_link"
                gps_msg.latitude  = float(vals[0])
                gps_msg.longitude = float(vals[1])
                gps_msg.altitude  = float(vals[2])
                gps_msg.status.status  = 0
                gps_msg.status.service = 1
                gps_msg.position_covariance = [
                    0.25, 0.0,  0.0,
                    0.0,  0.25, 0.0,
                    0.0,  0.0,  0.25]
                gps_msg.position_covariance_type = 2
                self._pub_gps.publish(gps_msg)
            except Exception as e:
                self._node.get_logger().warn(
                    f"GPS error: {e}", throttle_duration_sec=5)

        # -- RGB camera --------------------------------------------------------
        if self._camera:
            raw = self._camera.getImage()
            if raw:
                w = self._camera.getWidth()
                h = self._camera.getHeight()
                img_msg = Image()
                img_msg.header.stamp    = stamp
                img_msg.header.frame_id = "oakd_rgb_optical_link"
                img_msg.width    = w
                img_msg.height   = h
                img_msg.encoding = "bgra8"
                img_msg.step     = w * 4
                img_msg.data     = list(raw)
                self._pub_rgb.publish(img_msg)
                self._pub_cam_info.publish(
                    self._make_camera_info(
                        stamp, "oakd_rgb_optical_link", w, h))

        # -- Depth camera ------------------------------------------------------
        if self._depth:
            w = self._depth.getWidth()
            h = self._depth.getHeight()

            if w > 0 and h > 0:
                self._depth_ready = True

            if not self._depth_ready:
                return

            try:
                raw_d = self._depth.getRangeImage()
            except Exception:
                return

            if raw_d:
                depth_msg = Image()
                depth_msg.header.stamp    = stamp
                depth_msg.header.frame_id = "oakd_depth_optical_link"
                depth_msg.width    = w
                depth_msg.height   = h
                depth_msg.encoding = "32FC1"
                depth_msg.step     = w * 4
                depth_msg.data     = struct.pack(f"{w * h}f", *raw_d)

                if rclpy.ok():
                    self._pub_depth.publish(depth_msg)
                    self._pub_depth_info.publish(
                        self._make_camera_info(
                            stamp, "oakd_depth_optical_link", w, h))