import cv2
import time
from ultralytics import YOLO

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=1"
    )

def main():
    # Use a small model for Jetson (fast). Try: yolov8n.pt or yolov8s.pt
    model = YOLO("yolov8n.pt")

    cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("❌ Could not open CSI camera. Check nvarguscamerasrc / Jetson-IO / driver.")
        return

    prev_t = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        # Run YOLO
        results = model.predict(
            source=frame,
            imgsz=640,
            conf=0.35,
            iou=0.5,
            verbose=False,
            device=0,  # GPU (0). If issues, set to "cpu"
        )

        annotated = results[0].plot()

        # FPS calc
        now = time.time()
        dt = now - prev_t
        prev_t = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0)

        cv2.putText(
            annotated,
            f"FPS: {fps:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("YOLO - Jetson CSI", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):  # ESC or q
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()