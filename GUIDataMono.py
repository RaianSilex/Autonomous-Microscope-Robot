import os
import csv
import time
import threading
from datetime import datetime
from tkinter import Tk, StringVar, IntVar, N, S, E, W
from tkinter import ttk

# --- Cameras / Image ---
import PySpin
import cv2

try:
    # Pillow makes Tk image conversion easy; we'll fallback if missing
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

# --- Sensapex ---
from sensapex import UMP


# ---------------------------
# Configuration
# ---------------------------
AXIS_MIN = -10000
AXIS_MAX = +10000
SPEED_MIN = 10
SPEED_MAX = 2000
DEFAULT_STEP = 100
DEFAULT_SPEED_STEP = 50
LIVE_POLL_MS = 200           # UMP live poll
SEND_THROTTLE_MS = 60        # coalesce goto_pos
LOG_INTERVAL_MS = 500        # 0.5 s logging
CAM_UPDATE_MS = 30           # GUI refresh for camera
CAM_GET_TIMEOUT_MS = 1000    # PySpin image timeout
CAM_TEXT = "Blackfly S Live"


# ---------------------------
# Camera worker (PySpin in a thread)
# ---------------------------
class CameraWorker:
    def __init__(self):
        self.system = None
        self.cams = None
        self.cam = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.latest = None   # numpy array (Mono8)
        self.fps = 0.0

        # Video recording (per-run)
        self.recording = False
        self.video_path = None
        self.video_fps = 20
        self.video_writer = None

    def _set_stream_newest_only(self, cam):
        s_nm = cam.GetTLStreamNodeMap()
        handling = PySpin.CEnumerationPtr(s_nm.GetNode("StreamBufferHandlingMode"))
        if PySpin.IsAvailable(handling) and PySpin.IsWritable(handling):
            newest = handling.GetEntryByName("NewestOnly")
            if PySpin.IsAvailable(newest) and PySpin.IsReadable(newest):
                handling.SetIntValue(newest.GetValue())

    def start(self):
        if self.running:
            return True
        try:
            self.system = PySpin.System.GetInstance()
            self.cams = self.system.GetCameras()
            if self.cams.GetSize() == 0:
                self.stop()  # ensure cleanup
                return False

            self.cam = self.cams[0]
            self.cam.Init()

            # Stream & format
            self._set_stream_newest_only(self.cam)
            self.cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
            self.cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono8)

            # Manual exposure/gain defaults (best-effort)
            try:
                self.cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
            except PySpin.SpinnakerException:
                pass
            try:
                self.cam.GainAuto.SetValue(PySpin.GainAuto_Off)
            except PySpin.SpinnakerException:
                pass
            try:
                self.cam.ExposureTime.SetValue(20000.0)  # 20 ms
            except PySpin.SpinnakerException:
                pass
            try:
                self.cam.Gain.SetValue(min(5.0, self.cam.Gain.GetMax()))
            except PySpin.SpinnakerException:
                pass

            self.cam.BeginAcquisition()
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            return True
        except Exception:
            self.stop()
            return False

    def start_recording(self, video_path, video_fps=20):
        """Start writing frames to a video file for this run."""
        with self.lock:
            # If already recording, stop previous writer
            if self.recording and self.video_writer is not None:
                try:
                    self.video_writer.release()
                except Exception:
                    pass
                self.video_writer = None

            self.video_path = video_path
            self.video_fps = video_fps
            self.recording = True
            self.video_writer = None  # will be created on first frame

    def stop_recording(self):
        """Stop recording video for current run."""
        with self.lock:
            self.recording = False
            if self.video_writer is not None:
                try:
                    self.video_writer.release()
                except Exception:
                    pass
                self.video_writer = None
            self.video_path = None

    def _loop(self):
        last = time.time()
        while self.running:
            try:
                img = self.cam.GetNextImage(CAM_GET_TIMEOUT_MS)
                if img.IsIncomplete():
                    img.Release()
                    continue
                frame = img.GetNDArray()  # Mono8
                img.Release()

                # FPS estimate
                now = time.time()
                dt = max(1e-6, now - last)
                self.fps = 1.0 / dt
                last = now

                # Draw FPS text (in-place, white) for display / snapshots
                disp = cv2.putText(
                    frame.copy(),
                    f"~{self.fps:0.1f} fps",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    255,
                    1,
                    cv2.LINE_AA,
                )

                with self.lock:
                    # Update latest frame (with FPS overlay)
                    self.latest = disp

                    # --- Video recording (per-run) ---
                    if self.recording and self.video_path is not None:
                        # Convert Mono8 -> BGR for VideoWriter
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                        if self.video_writer is None:
                            h, w = frame_bgr.shape[:2]
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            self.video_writer = cv2.VideoWriter(
                                self.video_path, fourcc, self.video_fps, (w, h)
                            )
                        if self.video_writer is not None and self.video_writer.isOpened():
                            self.video_writer.write(frame_bgr)

            except PySpin.SpinnakerException:
                # brief sleep to avoid tight error loop
                time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def get_latest(self):
        with self.lock:
            if self.latest is None:
                return None
            return self.latest.copy()

    def stop(self):
        self.running = False
        try:
            if self.cam is not None:
                try:
                    self.cam.EndAcquisition()
                except Exception:
                    pass
        finally:
            # Release video writer
            try:
                self.stop_recording()
            except Exception:
                pass

            try:
                if self.cam is not None:
                    self.cam.DeInit()
            except Exception:
                pass
            try:
                if self.cams is not None:
                    self.cams.Clear()
            except Exception:
                pass
            try:
                if self.system is not None:
                    self.system.ReleaseInstance()
            except Exception:
                pass
            self.cam = None
            self.cams = None
            self.system = None
            self.thread = None


# ---------------------------
# Main GUI
# ---------------------------
class UMPGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensapex UMP + Blackfly S")

        # --- UMP init ---
        self.ump = UMP.get_ump()
        self.stage = self.ump.get_device(1)

        # Read initial live, use as starting target
        live = self._get_live_centered()
        if live is None:
            live = [0, 0, 0, 0]

        # Tk variables
        self.step = IntVar(value=DEFAULT_STEP)               # axis step
        self.speed_step = IntVar(value=DEFAULT_SPEED_STEP)   # speed step

        self.x = IntVar(value=live[0])
        self.y = IntVar(value=live[1])
        self.z = IntVar(value=live[2])
        self.d = IntVar(value=live[3])
        self.speed = IntVar(value=1000)

        self.live_x = StringVar(value="—")
        self.live_y = StringVar(value="—")
        self.live_z = StringVar(value="—")
        self.live_d = StringVar(value="—")
        self.live_speed = StringVar(value="—")

        self.status = StringVar(value="Ready")
        # Acquisition status text
        self.acq_status = StringVar(value="Data acquisition: STOPPED")

        # send coalescing + button repeats
        self._repeat_jobs = {}
        self._send_job_id = None

        # CSV logging
        self._log_writer = None
        self._log_file = None
        self._log_job_id = None

        # Trial / files (logs, frames, video)
        self.trial_name = None
        self.log_path = None
        self.frames_dir = None
        self.video_path = None
        self._frame_index = 0

        # Acquisition state
        self.acquiring = False

        # Camera (always running for live view; recording is per-run)
        self.cam_worker = CameraWorker()
        self._tkimg = None  # keep reference to prevent GC

        self._build_ui()
        self._bind_events()
        self._bind_keys()

        # Start background tasks
        self._poll_live()

        # Start camera
        ok = self.cam_worker.start()
        if not ok:
            self.status.set("Camera: not detected.")
        self._update_camera_view()

    # ---- Trial / path setup ----
    def _setup_new_trial(self):
        # Ensure base folders exist
        os.makedirs("logs", exist_ok=True)
        os.makedirs("saved_frames", exist_ok=True)
        os.makedirs("saved_videos", exist_ok=True)

        # Determine next trial number based on logs/trial_X.csv
        existing_trials = []
        for fname in os.listdir("logs"):
            if fname.startswith("trial_") and fname.endswith(".csv"):
                mid = fname[len("trial_"):-len(".csv")]
                if mid.isdigit():
                    existing_trials.append(int(mid))
        next_trial = max(existing_trials, default=0) + 1

        self.trial_name = f"trial_{next_trial}"
        # Paths
        self.log_path = os.path.join("logs", f"{self.trial_name}.csv")
        self.frames_dir = os.path.join("saved_frames", self.trial_name)
        self.video_path = os.path.join("saved_videos", f"{self.trial_name}.mp4")

        # Make frames folder for this run
        os.makedirs(self.frames_dir, exist_ok=True)

        # Reset frame index
        self._frame_index = 0

    # ---- Helpers ----
    def _center_to_device(self, v):
        return int(v + 10000)

    def _device_to_center(self, v):
        return int(v - 10000)

    def _get_live_centered(self):
        try:
            pos = self.stage.get_pos()  # [X,Y,Z,D]
            return [self._device_to_center(p) for p in pos[:4]]
        except Exception as e:
            self.status.set(f"Live read error: {e}")
            return None

    def _clamp(self, v, vmin, vmax):
        return max(vmin, min(vmax, v))

    # ---- UI ----
    def _build_ui(self):
        # Two main columns: left controls, right camera
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Left panel (controls)
        self.left = ttk.Frame(self.root, padding=12)
        self.left.grid(row=0, column=0, sticky=(N, S, E, W))
        for i in range(20):
            self.left.rowconfigure(i, weight=0)
        self.left.columnconfigure(0, weight=0)
        self.left.columnconfigure(1, weight=0)
        self.left.columnconfigure(2, weight=0)
        self.left.columnconfigure(3, weight=1)

        # Right panel (camera)
        self.right = ttk.Frame(self.root, padding=6)
        self.right.grid(row=0, column=1, sticky=(N, S, E, W))
        self.right.rowconfigure(1, weight=1)
        self.right.columnconfigure(0, weight=1)

        # Title
        self.title_lbl = ttk.Label(
            self.left, text="Sensapex UMP Controller", font=("Segoe UI", 14, "bold")
        )
        self.title_lbl.grid(row=0, column=0, columnspan=4, sticky=W, pady=(0, 8))

        # Steps row
        ttk.Label(self.left, text="Axis step:").grid(row=1, column=0, sticky=W)
        self.step_entry = ttk.Entry(self.left, width=7, textvariable=self.step, justify="right")
        self.step_entry.grid(row=1, column=1, sticky=W, padx=(6, 12))

        ttk.Label(self.left, text="Speed step:").grid(row=1, column=2, sticky=W)
        self.speed_step_entry = ttk.Entry(self.left, width=7, textvariable=self.speed_step, justify="right")
        self.speed_step_entry.grid(row=1, column=3, sticky=W)

        # Headers
        ttk.Label(self.left, text="Target (centered)", font=("Segoe UI", 10, "bold")).grid(
            row=2, column=3, sticky=W
        )
        ttk.Label(self.left, text="Live (centered)", font=("Segoe UI", 10, "bold")).grid(
            row=2, column=4, sticky=W
        )

        # Axis rows
        self.rows = {}
        r = 3
        for axis_name, var in [("X", self.x), ("Y", self.y), ("Z", self.z), ("D", self.d)]:
            self.rows[axis_name] = self._make_axis_row(
                self.left, r, axis_name, var, AXIS_MIN, AXIS_MAX, self.step
            )
            r += 1
        # Speed row (uses speed_step)
        self.speed_row = self._make_axis_row(
            self.left, r, "Speed", self.speed, SPEED_MIN, SPEED_MAX, self.speed_step
        )
        r += 1

        # --- Data Acquisition buttons ---
        self.btn_start_acq = ttk.Button(self.left, text="Start Data Acquisition",
                                        command=self._start_acquisition)
        self.btn_start_acq.grid(row=r, column=0, sticky=W, pady=(10, 4))

        self.btn_stop_acq = ttk.Button(self.left, text="Stop Data Acquisition",
                                       command=self._stop_acquisition)
        self.btn_stop_acq.grid(row=r, column=1, sticky=W, pady=(10, 4))

        # Acquisition status text (bigger so it's obvious)
        self.acq_status_lbl = ttk.Label(
            self.left, textvariable=self.acq_status, foreground="#006400", font=("Segoe UI", 10, "bold")
        )
        self.acq_status_lbl.grid(row=r, column=2, columnspan=2, sticky=W, pady=(10, 4))
        r += 1

        # Movement buttons
        self.btn_send = ttk.Button(self.left, text="Send Now", command=self._send_now)
        self.btn_send.grid(row=r, column=0, sticky=W, pady=(10, 4))
        self.btn_home = ttk.Button(self.left, text="Home (0,0,0,0)", command=self._home)
        self.btn_home.grid(row=r, column=1, sticky=W, pady=(10, 4))
        self.btn_sync = ttk.Button(self.left, text="Sync Target to Live", command=self._sync_target_to_live)
        self.btn_sync.grid(row=r, column=2, sticky=W, pady=(10, 4))
        self.btn_zero = ttk.Button(self.left, text="Calibrate Zero Here", command=self._calibrate_zero)
        self.btn_zero.grid(row=r, column=3, sticky=W, pady=(10, 4))
        r += 1

        # Keybinds info (font a bit bigger)
        keybinds_text = (
            "Keybinds:\n"
            "  X:  A (−), D (+)\n"
            "  Y:  S (−), W (+)\n"
            "  Z:  ↑ (+), ↓ (−)\n"
            "  D:  < or , (−), > or . (+)\n"
            "  Speed: ← (−), → (+)\n"
            "  Speed step: [ (−), ] (+)\n"
            "  Axis step:  O (−), P (+)"
        )
        self.keybinds_box = ttk.Label(
            self.left, text=keybinds_text, justify="left", font=("Segoe UI", 11)
        )
        self.keybinds_box.grid(row=r, column=0, columnspan=4, sticky=W, pady=(10, 4))
        r += 1

        # Status
        self.status_lbl = ttk.Label(self.left, textvariable=self.status, foreground="#333")
        self.status_lbl.grid(row=r, column=0, columnspan=4, sticky=W, pady=(8, 0))

        # --- Camera panel on the right ---
        ttk.Label(self.right, text=CAM_TEXT, font=("Segoe UI", 12, "bold")).grid(
            row=0, column=0, sticky=W, pady=(0, 6)
        )
        self.cam_label = ttk.Label(self.right)
        self.cam_label.grid(row=1, column=0, sticky=(N, S, E, W))  # expands

    def _make_axis_row(self, parent, row, name, var, vmin, vmax, step_var):
        lbl = ttk.Label(parent, text=name, width=8)
        up_btn = ttk.Button(parent, text="▲")
        dn_btn = ttk.Button(parent, text="▼")
        entry = ttk.Entry(parent, width=10, textvariable=var, justify="right")
        live_var = {
            "X": self.live_x, "Y": self.live_y, "Z": self.live_z,
            "D": self.live_d, "Speed": self.live_speed
        }[name]
        live_lbl = ttk.Label(parent, textvariable=live_var, width=12, anchor="e")

        key = f"{name}"

        def bump(delta):
            try:
                step = int(step_var.get())
            except Exception:
                step = DEFAULT_STEP if step_var is self.step else DEFAULT_SPEED_STEP
                step_var.set(step)
            var.set(self._clamp(var.get() + delta * step, vmin, vmax))
            self._schedule_send()

        def start_repeat(delta):
            bump(delta)

            def repeater():
                bump(delta)
                self._repeat_jobs[key] = self.root.after(70, repeater)
            self._repeat_jobs[key] = self.root.after(350, repeater)

        def stop_repeat(_evt=None):
            if key in self._repeat_jobs:
                self.root.after_cancel(self._repeat_jobs[key])
                del self._repeat_jobs[key]

        # ▲ increases, ▼ decreases
        up_btn.bind("<ButtonPress-1>", lambda _e: start_repeat(+1))
        up_btn.bind("<ButtonRelease-1>", stop_repeat)
        dn_btn.bind("<ButtonPress-1>", lambda _e: start_repeat(-1))
        dn_btn.bind("<ButtonRelease-1>", stop_repeat)

        # Layout
        lbl.grid(row=row, column=0, padx=(0, 6), sticky=W)
        up_btn.grid(row=row, column=1, padx=2, sticky=W)
        dn_btn.grid(row=row, column=2, padx=2, sticky=W)
        entry.grid(row=row, column=3, padx=(6, 6), sticky=W)
        live_lbl.grid(row=row, column=4, padx=(12, 0), sticky=W)

        # Clamp on manual edits
        def on_focus_out(_e):
            try:
                v = int(var.get())
            except Exception:
                v = 0
            var.set(self._clamp(v, vmin, vmax))
            self._schedule_send()
        entry.bind("<FocusOut>", on_focus_out)
        entry.bind("<Return>", on_focus_out)

        return {"var": var, "range": (vmin, vmax), "step_var": step_var}

    # ---- Keybinds ----
    def _bind_events(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _bind_keys(self):
        self.root.focus_force()
        self.root.bind_all("<KeyPress>", self._on_key_press)

    def _on_key_press(self, event):
        ks = (event.keysym or "").lower()
        mapping = {
            "up": ("Z", +1, self.step), "down": ("Z", -1, self.step),
            "a": ("X", -1, self.step), "d": ("X", +1, self.step),
            "s": ("Y", -1, self.step), "w": ("Y", +1, self.step),
            "less": ("D", -1, self.step), "comma": ("D", -1, self.step),
            "greater": ("D", +1, self.step), "period": ("D", +1, self.step),
            "left": ("Speed", -1, self.speed_step), "right": ("Speed", +1, self.speed_step),
        }
        # Step adjustments
        if ks == "bracketleft":   # [
            self._adjust_speed_step(-10); return
        if ks == "bracketright":  # ]
            self._adjust_speed_step(+10); return
        if ks == "o":
            self._adjust_axis_step(-10); return
        if ks == "p":
            self._adjust_axis_step(+10); return

        if ks in mapping:
            name, delta, which_step = mapping[ks]
            self._bump_axis(name, delta, which_step)

    def _bump_axis(self, name, delta, step_var):
        try:
            step = int(step_var.get())
        except Exception:
            step = DEFAULT_STEP if step_var is self.step else DEFAULT_SPEED_STEP
            step_var.set(step)
        if name == "Speed":
            var = self.speed
            vmin, vmax = SPEED_MIN, SPEED_MAX
        else:
            var = self.rows[name]["var"]
            vmin, vmax = self.rows[name]["range"]
        var.set(self._clamp(var.get() + delta * step, vmin, vmax))
        self._schedule_send()

    def _adjust_speed_step(self, delta):
        try:
            cur = int(self.speed_step.get())
        except Exception:
            cur = DEFAULT_SPEED_STEP
            self.speed_step.set(cur)
        new_val = max(1, cur + delta)
        self.speed_step.set(new_val)
        self.status.set(f"Speed step set to {new_val}")

    def _adjust_axis_step(self, delta):
        try:
            cur = int(self.step.get())
        except Exception:
            cur = DEFAULT_STEP
            self.step.set(cur)
        new_val = max(1, cur + delta)
        self.step.set(new_val)
        self.status.set(f"Axis step set to {new_val}")

    # ---- UMP Actions ----
    def _schedule_send(self):
        if self._send_job_id is not None:
            return
        self._send_job_id = self.root.after(SEND_THROTTLE_MS, self._send_now)

    def _send_now(self):
        self._send_job_id = None
        try:
            tgt = [self.x.get(), self.y.get(), self.z.get(), self.d.get()]
            dev = [self._center_to_device(v) for v in tgt]
            spd = self._clamp(self.speed.get(), SPEED_MIN, SPEED_MAX)
            self.speed.set(spd)
            self.stage.goto_pos(dev, speed=int(spd))
            self.status.set(
                f"Sent: X={tgt[0]}, Y={tgt[1]}, Z={tgt[2]}, D={tgt[3]} @ {spd} "
                f"(axis step {self.step.get()}, spd step {self.speed_step.get()})"
            )
        except Exception as e:
            self.status.set(f"Send error: {e}")

    def _home(self):
        self.x.set(0); self.y.set(0); self.z.set(0); self.d.set(0)
        self._send_now()

    def _sync_target_to_live(self):
        live = self._get_live_centered()
        if live is not None:
            self.x.set(live[0]); self.y.set(live[1]); self.z.set(live[2]); self.d.set(live[3])
            self.status.set("Targets synced to live.")
        else:
            self.status.set("Could not read live positions.")

    def _calibrate_zero(self):
        try:
            self.stage.calibrate_zero_position()
            self.status.set("Zero calibrated at current position.")
        except Exception as e:
            self.status.set(f"Calibrate error: {e}")

    def _poll_live(self):
        live = self._get_live_centered()
        if live is not None:
            self.live_x.set(f"{live[0]:d}")
            self.live_y.set(f"{live[1]:d}")
            self.live_z.set(f"{live[2]:d}")
            self.live_d.set(f"{live[3]:d}")
            self.live_speed.set(f"{self._clamp(self.speed.get(), SPEED_MIN, SPEED_MAX):d}")
        self.root.after(LIVE_POLL_MS, self._poll_live)

    # ---- Camera display ----
    def _update_camera_view(self):
        frame = self.cam_worker.get_latest()
        if frame is not None:
            # Convert Mono8 -> Tk image
            if PIL_AVAILABLE:
                img = Image.fromarray(frame)  # L mode (8-bit)
                # Fit to right panel while preserving aspect ratio
                w = self.right.winfo_width() or 640
                h = self.right.winfo_height() or 480
                h = max(50, h - 30)  # leave room for title
                img = img.copy()
                img.thumbnail((w, h))
                self._tkimg = ImageTk.PhotoImage(img)
            else:
                from tkinter import PhotoImage
                h, w = frame.shape[:2]
                header = f"P5 {w} {h} 255 ".encode("ascii")
                data = header + frame.tobytes()
                self._tkimg = PhotoImage(data=data)
            self.cam_label.configure(image=self._tkimg)
        self.root.after(CAM_UPDATE_MS, self._update_camera_view)

    # ---- Data acquisition control ----
    def _start_acquisition(self):
        # If already acquiring, end current run and start a fresh one
        if self.acquiring:
            self._stop_acquisition_internal()

        self._setup_new_trial()
        self._start_logging()
        # Start video recording for this run
        try:
            self.cam_worker.start_recording(self.video_path, video_fps=20)
        except Exception as e:
            self.status.set(f"Video start error: {e}")

        self.acquiring = True
        self.acq_status.set(f"Data acquisition: RUNNING ({self.trial_name})")
        self.status.set(f"Data acquisition started: {self.trial_name}")

    def _stop_acquisition(self):
        if not self.acquiring:
            self.status.set("Data acquisition already stopped.")
            return
        self._stop_acquisition_internal()
        self.status.set("Data acquisition stopped.")

    def _stop_acquisition_internal(self):
        self.acquiring = False
        self.acq_status.set("Data acquisition: STOPPED")

        # Cancel logging timer
        if self._log_job_id is not None:
            try:
                self.root.after_cancel(self._log_job_id)
            except Exception:
                pass
            self._log_job_id = None

        # Close log file
        if self._log_file is not None:
            try:
                self._log_file.flush()
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None
            self._log_writer = None

        # Stop video recording
        try:
            self.cam_worker.stop_recording()
        except Exception:
            pass

    # ---- Logging + frame saving ----
    def _start_logging(self):
        """Open CSV and start periodic logging for the current trial."""
        self._frame_index = 0
        self._log_file = open(self.log_path, mode="w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(["x", "y", "z", "d", "speed", "speed_step", "axis_step"])
        self._log_job_id = self.root.after(LOG_INTERVAL_MS, self._log_tick)

    def _log_tick(self):
        if not self.acquiring or not self._log_writer:
            return
        try:
            # CSV row
            row = [
                int(self.x.get()), int(self.y.get()), int(self.z.get()), int(self.d.get()),
                int(self.speed.get()), int(self.speed_step.get()), int(self.step.get())
            ]
            self._log_writer.writerow(row)
            self._log_file.flush()

            # Frame snapshot every 0.5 s, saved in saved_frames/trial_N
            frame = self.cam_worker.get_latest()
            if frame is not None and self.frames_dir is not None:
                fname = os.path.join(self.frames_dir, f"frame_{self._frame_index:06d}.png")
                try:
                    cv2.imwrite(fname, frame)
                    self._frame_index += 1
                except Exception as e:
                    self.status.set(f"Frame save error: {e}")
        except Exception as e:
            self.status.set(f"Log error: {e}")
        finally:
            if self.acquiring:
                # Schedule next tick only if still acquiring
                self._log_job_id = self.root.after(LOG_INTERVAL_MS, self._log_tick)

    # ---- Cleanup ----
    def _on_close(self):
        # Stop acquisition (if running)
        self._stop_acquisition_internal()
        # Stop camera
        try:
            self.cam_worker.stop()
        except Exception:
            pass
        # Cancel repeats / pending sends
        for _, after_id in list(self._repeat_jobs.items()):
            try:
                self.root.after_cancel(after_id)
            except Exception:
                pass
        if self._send_job_id:
            try:
                self.root.after_cancel(self._send_job_id)
            except Exception:
                pass
        self.root.destroy()


def main():
    root = Tk()
    # Make the window decently large so the camera has space
    root.geometry("1100x700")
    UMPGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()