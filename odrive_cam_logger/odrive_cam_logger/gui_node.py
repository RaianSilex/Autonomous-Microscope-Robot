import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

import numpy as np
import cv2

from tkinter import Tk, StringVar, IntVar, N, S, E, W
from tkinter import ttk

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

from .ros_interfaces import (
    TOPIC_CAM_IMAGE_COMPRESSED,
    TOPIC_MOTOR_LIVE,
    TOPIC_MOTOR_TARGET,
    SRV_ACQ_START,
    SRV_ACQ_STOP,
)

CAM_UPDATE_MS = 30


class ODriveCamGUI(Node):
    def __init__(self):
        super().__init__("gui_node")

        # ---- ROS ----
        self.sub_img = self.create_subscription(
            CompressedImage, TOPIC_CAM_IMAGE_COMPRESSED, self.on_image, 10
        )
        self.sub_motor_live = self.create_subscription(
            Int32, TOPIC_MOTOR_LIVE, self.on_motor_live, 10
        )

        self.pub_motor_target = self.create_publisher(Int32, TOPIC_MOTOR_TARGET, 10)

        self.cli_start = self.create_client(Trigger, SRV_ACQ_START)
        self.cli_stop = self.create_client(Trigger, SRV_ACQ_STOP)

        # ---- state ----
        self.latest_frame = None
        self.motor_live = None

        self.motor_jog_dir = 0  # -1,0,+1

        # ---- Tk ----
        self.root = Tk()
        self.root.title("ODrive + Camera Data Collection (No Sensapex)")
        self.root.geometry("1100x700")

        self.motor_target = IntVar(value=0)

        self.live_motor_str = StringVar(value="—")
        self.target_motor_str = StringVar(value="—")

        self.status = StringVar(value="Ready")
        self.acq_status = StringVar(value="Data acquisition: STOPPED")

        self._tkimg = None

        self._build_ui()
        self._bind_events()

        self.root.after(100, self._update_motor_widgets)
        self.root.after(CAM_UPDATE_MS, self._update_camera_view)

        # publish while jogging
        self.ctrl_timer = self.create_timer(0.1, self.control_step)

    # ---------------- ROS callbacks ----------------
    def on_motor_live(self, msg: Int32):
        self.motor_live = int(msg.data)

    def on_image(self, msg: CompressedImage):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if frame is not None:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f"Image decode error: {e}")

    # ---------------- motor control ----------------
    def control_step(self):
        if self.motor_jog_dir == 0:
            return
        if self.motor_live is None:
            return

        # simple jog by incrementing target
        step_counts = 500 * self.motor_jog_dir
        new_target = int(self.motor_target.get()) + step_counts
        self.motor_target.set(new_target)
        self.pub_motor_target.publish(Int32(data=int(new_target)))
        self.target_motor_str.set(str(int(new_target)))

    # ---------------- GUI ----------------
    def _build_ui(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.left = ttk.Frame(self.root, padding=12)
        self.left.grid(row=0, column=0, sticky=(N, S, E, W))

        self.right = ttk.Frame(self.root, padding=6)
        self.right.grid(row=0, column=1, sticky=(N, S, E, W))
        self.right.rowconfigure(1, weight=1)
        self.right.columnconfigure(0, weight=1)

        ttk.Label(self.left, text="ODrive + Camera Data Collection", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, columnspan=4, sticky=W, pady=(0, 10)
        )

        # Motor controls
        ttk.Label(self.left, text="Motor", width=8).grid(row=1, column=0, sticky=W)

        self.btn_up = ttk.Button(self.left, text="▲")
        self.btn_dn = ttk.Button(self.left, text="▼")
        self.btn_up.grid(row=1, column=1, padx=2, sticky=W)
        self.btn_dn.grid(row=1, column=2, padx=2, sticky=W)

        self.motor_entry = ttk.Entry(self.left, width=10, textvariable=self.motor_target, justify="right")
        self.motor_entry.grid(row=1, column=3, padx=(6, 6), sticky=W)

        # Target + Live readouts
        ttk.Label(self.left, text="Target:", width=8).grid(row=2, column=0, sticky=W, pady=(8, 0))
        ttk.Label(self.left, textvariable=self.target_motor_str, width=12, anchor="e").grid(
            row=2, column=1, columnspan=3, sticky=W, pady=(8, 0)
        )

        ttk.Label(self.left, text="Live:", width=8).grid(row=3, column=0, sticky=W, pady=(2, 0))
        ttk.Label(self.left, textvariable=self.live_motor_str, width=12, anchor="e").grid(
            row=3, column=1, columnspan=3, sticky=W, pady=(2, 0)
        )

        # Acquisition
        self.btn_start = ttk.Button(self.left, text="Start Data Acquisition", command=self._start_acq)
        self.btn_stop = ttk.Button(self.left, text="Stop Data Acquisition", command=self._stop_acq)
        self.btn_start.grid(row=4, column=0, sticky=W, pady=(16, 4))
        self.btn_stop.grid(row=4, column=1, sticky=W, pady=(16, 4))

        ttk.Label(self.left, textvariable=self.acq_status, foreground="#006400",
                  font=("Segoe UI", 10, "bold")).grid(
            row=4, column=2, columnspan=2, sticky=W, pady=(16, 4)
        )

        # Send once
        self.btn_send = ttk.Button(self.left, text="Send Motor Target Now", command=self._send_motor_now)
        self.btn_send.grid(row=5, column=0, sticky=W, pady=(10, 4))

        ttk.Label(self.left, textvariable=self.status, foreground="#333").grid(
            row=6, column=0, columnspan=4, sticky=W, pady=(10, 0)
        )

        # Camera
        ttk.Label(self.right, text="Blackfly Live", font=("Segoe UI", 12, "bold")).grid(
            row=0, column=0, sticky=W, pady=(0, 6)
        )
        self.cam_label = ttk.Label(self.right)
        self.cam_label.grid(row=1, column=0, sticky=(N, S, E, W))

        # bindings
        self.btn_up.bind("<ButtonPress-1>", lambda _e: self._start_jog(+1))
        self.btn_up.bind("<ButtonRelease-1>", self._stop_jog)
        self.btn_dn.bind("<ButtonPress-1>", lambda _e: self._start_jog(-1))
        self.btn_dn.bind("<ButtonRelease-1>", self._stop_jog)

        self.motor_entry.bind("<Return>", lambda _e: self._send_motor_now())
        self.motor_entry.bind("<FocusOut>", lambda _e: self._send_motor_now())

    def _bind_events(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _update_motor_widgets(self):
        if self.motor_live is not None:
            self.live_motor_str.set(str(self.motor_live))

            # init target to live once at startup
            if self.target_motor_str.get() == "—" and int(self.motor_target.get()) == 0:
                self.motor_target.set(int(self.motor_live))
                self.target_motor_str.set(str(int(self.motor_live)))

        self.root.after(100, self._update_motor_widgets)

    def _start_jog(self, direction: int):
        if self.motor_live is not None:
            self.motor_target.set(int(self.motor_live))
            self.target_motor_str.set(str(int(self.motor_live)))
        self.motor_jog_dir = int(direction)
        self.status.set("Jogging motor... (hold button)")

    def _stop_jog(self, _evt=None):
        self.motor_jog_dir = 0
        self.status.set("Jog stopped.")
        self._send_motor_now()

    def _send_motor_now(self):
        try:
            target = int(self.motor_target.get())
            self.pub_motor_target.publish(Int32(data=target))
            self.target_motor_str.set(str(target))
            self.status.set(f"Sent motor target: {target}")
        except Exception as e:
            self.status.set(f"Motor send error: {e}")

    def _update_camera_view(self):
        frame = self.latest_frame
        if frame is not None and PIL_AVAILABLE:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)

                w = self.right.winfo_width() or 640
                h = self.right.winfo_height() or 480
                h = max(50, h - 30)

                img = img.copy()
                img.thumbnail((w, h))
                self._tkimg = ImageTk.PhotoImage(img)
                self.cam_label.configure(image=self._tkimg)
            except Exception:
                pass
        elif frame is not None and not PIL_AVAILABLE:
            self.cam_label.configure(text="Install pillow for camera view")

        self.root.after(CAM_UPDATE_MS, self._update_camera_view)

    # --------- acquisition service calls ----------
    def _start_acq(self):
        if not self.cli_start.service_is_ready():
            self.status.set("Waiting for /acq/start service...")
            self.cli_start.wait_for_service(timeout_sec=2.0)

        req = Trigger.Request()

        def _call():
            try:
                future = self.cli_start.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None and future.result().success:
                    self.acq_status.set("Data acquisition: RUNNING")
                    self.status.set(future.result().message)
                else:
                    self.acq_status.set("Data acquisition: FAILED")
                    self.status.set("Start failed (no response).")
            except Exception as e:
                self.acq_status.set("Data acquisition: FAILED")
                self.status.set(f"Start error: {e}")

        threading.Thread(target=_call, daemon=True).start()

    def _stop_acq(self):
        if not self.cli_stop.service_is_ready():
            self.status.set("Waiting for /acq/stop service...")
            self.cli_stop.wait_for_service(timeout_sec=2.0)

        req = Trigger.Request()

        def _call():
            try:
                future = self.cli_stop.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None and future.result().success:
                    self.acq_status.set("Data acquisition: STOPPED")
                    self.status.set(future.result().message)
                else:
                    self.status.set("Stop failed (no response).")
            except Exception as e:
                self.status.set(f"Stop error: {e}")

        threading.Thread(target=_call, daemon=True).start()

    def _on_close(self):
        try:
            self.motor_jog_dir = 0
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        self.root.mainloop()


def main():
    rclpy.init()
    node = ODriveCamGUI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()