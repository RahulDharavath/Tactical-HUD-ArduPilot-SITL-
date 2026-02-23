# main.py
import cv2
import time
from _1_telemetry import TelemetryHandler
from _2_detection import Detector
from _3_tracker import TargetSelector
from _4_hud import HUDRenderer

# Initialize modules
telemetry = TelemetryHandler()
detector = Detector()
selector = TargetSelector(detector.model)   
hud = HUDRenderer()

cap = cv2.VideoCapture("ss.mp4")

if not cap.isOpened():
    print("Video error")
    exit()

frame_count = 0
skip_frames = 2
last_results = None
fps = 0

while True:
    start = time.time()

    # 1. Read frame
    ret, frame = cap.read()
    if not ret:
        break

    # 2. Resize for speed
    frame = cv2.resize(frame, (416, 416))
    frame_count += 1

    # 3. Detection with skip logic
    if frame_count % skip_frames == 0:
        results = detector.detect(frame)
        last_results = results
    else:
        results = last_results

    annotated = frame.copy()
    target_box = None

    # 4. Draw detection boxes
    if results is not None:

        for box in results[0].boxes:

            # Bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Confidence
            conf = float(box.conf[0])

            # Class
            cls = int(box.cls[0])
            class_name = detector.model.names[cls]

            # Track ID 
            if box.id is not None:
                track_id = int(box.id[0])
            else:
                track_id = -1

            # Label with track ID
            label = f"ID:{track_id} {class_name} {conf:.2f}"

            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            cv2.putText(
                annotated,
                label,
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        # Select closest vehicle
        target_box = selector.select_target(results, frame.shape)

    # 5. Telemetry update
    telem_data = telemetry.update()

    # 6. Draw HUD
    hud.draw_crosshair(annotated)
    hud.draw_telemetry(annotated, telem_data)
    hud.draw_timestamp(annotated)
    hud.highlight_target(annotated, target_box)

    # 7. FPS Calculation
    new_fps = 1 / (time.time() - start)
    fps = 0.9 * fps + 0.1 * new_fps

    cv2.putText(
        annotated,
        f"FPS: {fps:.2f}",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )

    # 8. Display
    cv2.imshow("AI Tactical HUD", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()