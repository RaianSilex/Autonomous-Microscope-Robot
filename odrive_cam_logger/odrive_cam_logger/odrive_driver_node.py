import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL

from .ros_interfaces import TOPIC_MOTOR_LIVE, TOPIC_MOTOR_TARGET


class ODriveDriverNode(Node):
    def __init__(self):
        super().__init__("odrive_driver_node")

        self.declare_parameter("poll_ms", 50)
        self.declare_parameter("goto_speed_turns_s", 0.5)
        self.declare_parameter("deadband_counts", 200)

        self.pub_live = self.create_publisher(Int32, TOPIC_MOTOR_LIVE, 10)
        self.sub_target = self.create_subscription(Int32, TOPIC_MOTOR_TARGET, self.on_target, 10)

        self.target_counts = None

        self.odrv = None
        self.axis = None
        self.connected = False

        self._connect()

        poll_ms = int(self.get_parameter("poll_ms").value)
        self.timer = self.create_timer(max(0.01, poll_ms / 1000.0), self.tick)

    def _connect(self):
        try:
            self.get_logger().info("Connecting to ODrive...")
            self.odrv = odrive.find_any()
            self.axis = self.odrv.axis0

            self.odrv.clear_errors()
            time.sleep(0.3)

            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.8)

            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.axis.controller.input_vel = 0.0

            self.connected = True
            try:
                cur = int(self.axis.encoder.shadow_count)
                self.get_logger().info(f"ODrive connected. axis0 current_counts={cur}")
            except Exception:
                self.get_logger().info("ODrive connected. axis0 current_counts=?")
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"ODrive connect failed: {e}")

    def on_target(self, msg: Int32):
        # Optional control: if you only log, just never publish to /motor/target_counts
        self.target_counts = int(msg.data)

    def tick(self):
        if not self.connected or self.axis is None:
            return

        # publish live encoder counts
        try:
            pos_counts = int(self.axis.encoder.shadow_count)
            self.pub_live.publish(Int32(data=pos_counts))
        except Exception as e:
            self.get_logger().warn(f"Read encoder failed: {e}")
            return

        # optional goto (velocity) behavior toward target
        if self.target_counts is not None:
            try:
                target = int(self.target_counts)
                err = target - pos_counts
                deadband = int(self.get_parameter("deadband_counts").value)

                if abs(err) <= deadband:
                    self.axis.controller.input_vel = 0.0
                else:
                    v = float(self.get_parameter("goto_speed_turns_s").value)
                    self.axis.controller.input_vel = v * (1.0 if err > 0 else -1.0)
            except Exception as e:
                self.get_logger().warn(f"Motor control failed: {e}")

    def destroy_node(self):
        try:
            if self.connected and self.axis is not None:
                self.axis.controller.input_vel = 0.0
                self.axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ODriveDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()