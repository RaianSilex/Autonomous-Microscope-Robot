import os
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from .ros_interfaces import (
    TOPIC_MOTOR_LIVE,
    TOPIC_MOTOR_TARGET,
    TOPIC_CAM_IMAGE_COMPRESSED,
    TOPIC_CAM_REC_CMD,
    SRV_ACQ_START,
    SRV_ACQ_STOP,
)


class LoggerNode(Node):
    def __init__(self):
        super().__init__("logger_node")

        self.declare_parameter("log_interval_ms", 500)

        # latest values
        self.latest_motor_live = None
        self.latest_motor_target = None
        self.latest_img = None

        # acquisition state
        self.acquiring = False
        self.trial_name = None
        self.trial_dir = None
        self.frames_dir = None
        self.video_path = None
        self.log_path = None

        self.frame_index = 0
        self.log_file = None
        self.writer = None
        self.t0 = None

        # subs
        self.sub_motor_live = self.create_subscription(Int32, TOPIC_MOTOR_LIVE, self.on_motor_live, 10)
        self.sub_motor_target = self.create_subscription(Int32, TOPIC_MOTOR_TARGET, self.on_motor_target, 10)
        self.sub_img = self.create_subscription(CompressedImage, TOPIC_CAM_IMAGE_COMPRESSED, self.on_img, 10)

        # pub (record start/stop)
        self.pub_rec_cmd = self.create_publisher(String, TOPIC_CAM_REC_CMD, 10)

        # services
        self.srv_start = self.create_service(Trigger, SRV_ACQ_START, self.on_start)
        self.srv_stop = self.create_service(Trigger, SRV_ACQ_STOP, self.on_stop)

        interval = float(self.get_parameter("log_interval_ms").value) / 1000.0
        self.timer = self.create_timer(max(0.01, interval), self.tick)

        self.get_logger().info("Logger node started (camera + ODrive).")

    # ---------------- callbacks ----------------
    def on_motor_live(self, msg: Int32):
        self.latest_motor_live = int(msg.data)

    def on_motor_target(self, msg: Int32):
        self.latest_motor_target = int(msg.data)

    def on_img(self, msg: CompressedImage):
        self.latest_img = msg

    # ---------------- paths ----------------
    def _base_data_dir(self) -> str:
        # Save datasets to: ~/.ros/odrive_cam_logger/data
        return os.path.join(os.path.expanduser("~"), ".ros", "odrive_cam_logger", "data")

    def _setup_trial(self):
        base_dir = self._base_data_dir()
        os.makedirs(base_dir, exist_ok=True)

        existing = []
        for name in os.listdir(base_dir):
            if name.startswith("trial_"):
                mid = name[len("trial_"):]
                if mid.isdigit():
                    existing.append(int(mid))
        next_trial = max(existing, default=0) + 1

        self.trial_name = f"trial_{next_trial}"
        self.trial_dir = os.path.join(base_dir, self.trial_name)
        self.frames_dir = os.path.join(self.trial_dir, "frames")

        os.makedirs(self.frames_dir, exist_ok=True)

        self.log_path = os.path.join(self.trial_dir, "log.csv")
        self.video_path = os.path.join(self.trial_dir, "video.mp4")
        self.frame_index = 0

        self.get_logger().info(f"Trial directory: {self.trial_dir}")

    def _open_csv(self):
        self.log_file = open(self.log_path, "w", newline="")
        self.writer = csv.writer(self.log_file)

        # ✅ now includes target_motor + current_motor
        self.writer.writerow([
            "stamp_sec",
            "stamp_nanosec",
            "t_elapsed_sec",
            "target_motor_counts",
            "current_motor_counts",
            "frame_index",
        ])

    # ---------------- services ----------------
    def on_start(self, _req, res):
        if self.acquiring:
            res.success = True
            res.message = "Already acquiring."
            return res

        self._setup_trial()
        self._open_csv()

        self.t0 = self.get_clock().now()

        # tell camera node to record into this trial folder (optional)
        self.pub_rec_cmd.publish(String(data=self.video_path))

        self.acquiring = True
        res.success = True
        res.message = f"Acquisition started: {self.trial_name}"
        self.get_logger().info(res.message)
        return res

    def on_stop(self, _req, res):
        if not self.acquiring:
            res.success = True
            res.message = "Already stopped."
            return res

        self.acquiring = False
        self.t0 = None

        # stop recording
        self.pub_rec_cmd.publish(String(data=""))

        try:
            if self.log_file:
                self.log_file.flush()
                self.log_file.close()
        except Exception:
            pass
        self.log_file = None
        self.writer = None

        res.success = True
        res.message = "Acquisition stopped."
        self.get_logger().info(res.message)
        return res

    # ---------------- periodic logging ----------------
    def tick(self):
        if not self.acquiring or self.writer is None:
            return
        if self.latest_img is None or self.t0 is None:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()
        t_elapsed = (now - self.t0).nanoseconds * 1e-9

        # if target not received yet, log as current (or 0)
        cur = int(self.latest_motor_live) if self.latest_motor_live is not None else 0
        tgt = int(self.latest_motor_target) if self.latest_motor_target is not None else cur

        # save frame
        try:
            data = np.frombuffer(self.latest_img.data, dtype=np.uint8)
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if frame is not None:
                fname = os.path.join(self.frames_dir, f"frame_{self.frame_index:06d}.jpg")
                cv2.imwrite(fname, frame)
        except Exception as e:
            self.get_logger().warn(f"Frame save error: {e}")

        # write CSV row
        try:
            self.writer.writerow([
                now_msg.sec,
                now_msg.nanosec,
                float(t_elapsed),
                tgt,
                cur,
                int(self.frame_index),
            ])
            self.log_file.flush()
        except Exception as e:
            self.get_logger().warn(f"CSV write error: {e}")

        self.frame_index += 1


def main():
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()