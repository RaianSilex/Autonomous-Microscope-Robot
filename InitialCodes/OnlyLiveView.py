import PySpin
import cv2
import time

WIN = "Blackfly S Live"

def set_stream_newest_only(cam):
    s_nm = cam.GetTLStreamNodeMap()
    handling = PySpin.CEnumerationPtr(s_nm.GetNode("StreamBufferHandlingMode"))
    if PySpin.IsAvailable(handling) and PySpin.IsWritable(handling):
        newest = handling.GetEntryByName("NewestOnly")
        if PySpin.IsAvailable(newest) and PySpin.IsReadable(newest):
            handling.SetIntValue(newest.GetValue())

def live():
    system = PySpin.System.GetInstance()
    cams = system.GetCameras()
    if cams.GetSize() == 0:
        print("❌ No FLIR cameras detected.")
        system.ReleaseInstance()
        return

    cam = cams[0]
    print("✅ Using:", cam.TLDevice.DeviceModelName.ToString(),
          "SN:", cam.TLDevice.DeviceSerialNumber.ToString())

    try:
        cam.Init()
        nm = cam.GetNodeMap()

        # Stream: drop old frames if UI lags
        set_stream_newest_only(cam)

        # Continuous acquisition, Mono8
        cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono8)

        # Manual exposure/gain (good for microscopes)
        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        cam.GainAuto.SetValue(PySpin.GainAuto_Off)

        # Start values (microseconds, dB)
        try:
            cam.ExposureTime.SetValue(20000.0)   # 20 ms
        except PySpin.SpinnakerException:
            pass
        try:
            cam.Gain.SetValue(min(5.0, cam.Gain.GetMax()))
        except PySpin.SpinnakerException:
            pass

        cam.BeginAcquisition()
        print("🎥 Live view running.")
        print("Keys: q/ESC quit | s save PNG | +/- exposure | [ ] gain | a toggle auto-exposure")

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        last = time.time()
        exp_auto_on = False

        while True:
            img = cam.GetNextImage(1000)   # 1 s timeout
            if img.IsIncomplete():
                img.Release()
                continue

            frame = img.GetNDArray()
            img.Release()

            # Simple FPS estimate
            now = time.time()
            fps = 1.0 / max(1e-6, (now - last))
            last = now
            disp = frame.copy()
            cv2.putText(disp, f"~{fps:0.1f} fps", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1, cv2.LINE_AA)

            cv2.imshow(WIN, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
            elif k == ord('s'):
                cv2.imwrite("live_frame.png", frame)
                print("💾 Saved live_frame.png")
            elif k in (ord('+'), ord('=')):  # increase exposure
                try:
                    val = cam.ExposureTime.GetValue()
                    cam.ExposureTime.SetValue(min(val * 1.2, cam.ExposureTime.GetMax()))
                    print(f"Exposure: {cam.ExposureTime.GetValue():.0f} us")
                except PySpin.SpinnakerException:
                    pass
            elif k == ord('-'):  # decrease exposure
                try:
                    val = cam.ExposureTime.GetValue()
                    cam.ExposureTime.SetValue(max(val / 1.2, cam.ExposureTime.GetMin()))
                    print(f"Exposure: {cam.ExposureTime.GetValue():.0f} us")
                except PySpin.SpinnakerException:
                    pass
            elif k == ord('['):  # decrease gain
                try:
                    val = cam.Gain.GetValue()
                    cam.Gain.SetValue(max(val - 1.0, cam.Gain.GetMin()))
                    print(f"Gain: {cam.Gain.GetValue():.1f} dB")
                except PySpin.SpinnakerException:
                    pass
            elif k == ord(']'):  # increase gain
                try:
                    val = cam.Gain.GetValue()
                    cam.Gain.SetValue(min(val + 1.0, cam.Gain.GetMax()))
                    print(f"Gain: {cam.Gain.GetValue():.1f} dB")
                except PySpin.SpinnakerException:
                    pass
            elif k == ord('a'):  # toggle auto-exposure
                exp_auto_on = not exp_auto_on
                try:
                    cam.ExposureAuto.SetValue(
                        PySpin.ExposureAuto_Continuous if exp_auto_on else PySpin.ExposureAuto_Off
                    )
                    print("ExposureAuto:", "On" if exp_auto_on else "Off")
                except PySpin.SpinnakerException:
                    pass

        cam.EndAcquisition()
        cv2.destroyAllWindows()

    finally:
        cam.DeInit()
        cams.Clear()
        system.ReleaseInstance()

if __name__ == "__main__":
    live()

    print("Hello")