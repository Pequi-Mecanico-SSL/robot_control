#!/usr/bin/env python3
import math
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MimoPidOmni(Node):
    def __init__(self):
        super().__init__("mimo_pid_omni")

        # --------- Robot selection ----------
        self.robot_id = self.declare_parameter("robot_id", 0).value
        self.color = self.declare_parameter("color", "blue").value
        # Angle where each wheel is located (0 is to the right of robot)
        default_wheel_orientation = [
            math.pi / 4,
            3 * math.pi / 4,
            5 * math.pi / 4,
            7 * math.pi / 4,
        ]
        self.wheel_orientation: List[float] = self.declare_parameter(
            "wheel_orientation", default_wheel_orientation
        ).value
        self.robot_radius = float(self.declare_parameter("robot_radius", 0.09).value)  # [m]

        # --------- Useful extra params ----------
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 100.0).value)
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.03).value)  # [m]
        self.max_wheel_speed = float(self.declare_parameter("max_wheel_speed", 50.0).value)  # [rad/s]
        self.pwm_max = float(self.declare_parameter("pwm_max", 50.0).value)  # publish in [-pwm_max, pwm_max]
        self.pwm_deadband = float(self.declare_parameter("pwm_deadband", 0.0).value)

        # PID gains as 3x3 (MIMO). Defaults are diagonal.
        kp_default = [1.0, 1.0, 0.5]
        ki_default = [0.0, 0.0, 0.0]
        kd_default = [0.0, 0.0, 0.0]

        self.Kp =   [[kp_default[0], 0.0, 0.0],
                    [0.0, kp_default[1], 0.0],
                    [0.0, 0.0, kp_default[2]]]

        self.Ki =   [[ki_default[0], 0.0, 0.0],
                    [0.0, ki_default[1], 0.0],
                    [0.0, 0.0, ki_default[2]]]

        self.Kd =   [[kd_default[0], 0.0, 0.0],
                    [0.0, kd_default[1], 0.0],
                    [0.0, 0.0, kd_default[2]]]

        # self.Kp = np.array(self.declare_parameter(
        #     "Kp",
        #     [[kp_default[0], 0.0, 0.0],
        #      [0.0, kp_default[1], 0.0],
        #      [0.0, 0.0, kp_default[2]]]
        # ).value, dtype=float)

        # self.Ki = np.array(self.declare_parameter(
        #     "Ki",
        #     [[ki_default[0], 0.0, 0.0],
        #      [0.0, ki_default[1], 0.0],
        #      [0.0, 0.0, ki_default[2]]]
        # ).value, dtype=float)

        # self.Kd = np.array(self.declare_parameter(
        #     "Kd",
        #     [[kd_default[0], 0.0, 0.0],
        #      [0.0, kd_default[1], 0.0],
        #      [0.0, 0.0, kd_default[2]]]
        # ).value, dtype=float)

        # Derivative low-pass (0=no filtering, ->1 stronger smoothing)
        self.deriv_alpha = float(self.declare_parameter("deriv_alpha", 0.7).value)
        # Integral clamp (on twist correction norm)
        self.int_limit = float(self.declare_parameter("integrator_limit", 2.0).value)

        # Optional feedforward scale on desired twist before mapping to wheels
        self.ff_scale = float(self.declare_parameter("feedforward_scale", 1.0).value)

        # Optional velocity smoothing (1st-order low-pass) for measured twist
        self.meas_alpha = float(self.declare_parameter("meas_alpha", 0.0).value)

        # --------- Topics ----------
        ns_piece = f"{self.color}/robot{self.robot_id}"
        self.sub_meas = self.create_subscription(
            Twist, f"/simulator/velocity/{ns_piece}", self._on_meas, 10
        )
        self.sub_cmd = self.create_subscription(
            Twist, f"/pid/cmd/velocity/{ns_piece}", self._on_cmd, 10
        )
        self.pub_pwm = self.create_publisher(
            Float32MultiArray, f"/simulator/cmd/wheel/{ns_piece}", 10
        )

        # --------- State ----------
        self._v_meas = np.zeros(3)       # [vx, vy, wz]
        self._v_meas_filt = np.zeros(3)
        self._v_cmd = np.zeros(3)        # desired [vx, vy, wz]
        self._err_prev = np.zeros(3)
        self._derr_filt = np.zeros(3)
        self._i_term = np.zeros(3)

        self._last_update: Optional[Time] = None

        # Precompute allocation matrix M (4x3): wheel ang vel = M * body_twist
        # For omniwheel placed at angle phi_i around a circle:
        # t_i = [-sin(phi_i), cos(phi_i)]  (tangent direction)
        # w_i = ( t_i · [vx, vy] + wz * robot_radius ) / wheel_radius
        M_rows = []
        for phi in self.wheel_orientation:
            sx, cx = math.sin(phi), math.cos(phi)
            M_rows.append([-sx, cx, self.robot_radius])
        self.M = np.array(M_rows, dtype=float) / self.wheel_radius  # shape (4,3)

        # Control loop timer
        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self._control_cb)
        self.get_logger().info(
            f"MIMO PID Omni controller up: topics in=/velocity{ns_piece}, /cmd/velocity{ns_piece}; out=/cmd{ns_piece}"
        )

    # -------------------- Callbacks --------------------
    def _on_meas(self, msg: Twist):
        # self.get_logger().info(f"Vel: [{msg.linear.x:.2f},\t{msg.linear.y:.2f},\t{msg.angular.z:.2f}]")
        raw = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)
        if self.meas_alpha > 0.0:
            self._v_meas_filt = self.meas_alpha * self._v_meas_filt + (1.0 - self.meas_alpha) * raw
            self._v_meas = self._v_meas_filt
        else:
            self._v_meas = raw

    def _on_cmd(self, msg: Twist):
        # self.get_logger().info(f"CMD: [{msg.linear.x:.2f},\t{msg.linear.y:.2f},\t{msg.angular.z:.2f}]")
        self._v_cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

    # -------------------- Control --------------------
    def _control_cb(self):
        now = self.get_clock().now()
        if self._last_update is None:
            self._last_update = now
            return

        dt = (now - self._last_update).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = self.dt
        self._last_update = now

        # Error in body twist
        e = self._v_cmd - self._v_meas
        self.get_logger().info(f"Error: [{e[0]:.2f},\t{e[1]:.2f},\t{e[2]:.2f}]")

        # Integrator with clamp (anti-windup)
        self._i_term += e * dt
        i_norm = np.linalg.norm(self._i_term)
        if i_norm > self.int_limit > 0.0:
            self._i_term *= self.int_limit / i_norm

        # Derivative with low-pass
        derr = (e - self._err_prev) / max(dt, 1e-6)
        self._derr_filt = self.deriv_alpha * self._derr_filt + (1.0 - self.deriv_alpha) * derr
        self._err_prev = e

        # PID output in body-twist space (3x1)
        u_pid = self.Kp @ e + self.Ki @ self._i_term + self.Kd @ self._derr_filt

        # Feed-forward (desired twist) + feedback correction
        v_out = self.ff_scale * self._v_cmd + u_pid

        # Map to wheel angular speeds (4x1)
        w = self.M @ v_out

        # Saturate wheel speeds, then map to PWM in [-pwm_max, pwm_max]
        w_sat = np.clip(w, -self.max_wheel_speed, self.max_wheel_speed)
        pwm = (w_sat / self.max_wheel_speed) * self.pwm_max

        # Apply deadband (optional)
        if self.pwm_deadband > 0.0:
            for i in range(len(pwm)):
                if abs(pwm[i]) < self.pwm_deadband:
                    pwm[i] = 0.0

        # self.get_logger().info(f"PWM: {pwm}")

        # Publish
        msg = Float32MultiArray()
        msg.data = pwm.astype(np.float32).tolist()
        self.pub_pwm.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MimoPidOmni()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
