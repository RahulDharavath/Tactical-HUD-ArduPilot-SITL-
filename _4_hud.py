# hud.py
import cv2
import datetime

class HUDRenderer:

    def draw_crosshair(self, frame):
        h, w, _ = frame.shape       # Get frame dimensions (h=720, w=1280, BGR=3)
        cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (0,255,0), 1) # Horizontal line
        cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (0,255,0), 1) # Vertical line

    def draw_telemetry(self, frame, telemetry):
        y = 20  # at 20px (top-left on the screen)
        for key, value in telemetry.items():  #Loop through dict: {'alt':125, 'speed':30,..
            text = f"{key.upper()}: {value}"  # make all capital letters -> 'ALT':125, ...
            cv2.putText(frame, text, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, # Clean font
                        0.5, (0,255,0), 1)        # size=0.5, BGR, 1px thick  
            y += 20                               # Move down 20px for next line

    def draw_timestamp(self, frame):
        now = datetime.datetime.now().strftime("%H:%M:%S")  # current time
        cv2.putText(frame, now, (10, frame.shape[0]-10),    # write at bottom left
                    cv2.FONT_HERSHEY_SIMPLEX,               # clean font
                    0.5, (0,255,0), 1)                      # size=0.5, BGR, thick=1

    def highlight_target(self, frame, box):
        if box is None:              # No target selected
            return                   # Skip drawing

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        obj_id = int(box.id[0])             # Track ID: 1, 2, 3...
        conf = float(box.conf[0])           # Confidence: 0.92

        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,255), 2)     # RED thick box
        cv2.putText(frame, f"LOCKED ID:{obj_id} {conf:.2f}",     # "LOCKED ID:1  0.92"
                    (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0,0,255), 2)
        



# --------------------------------------------------
# crosshair explanation:
# Frame: 1280x720 → h=720, w=1280, channels=3 (BGR)

# Horizontal line: left To right
            # cv2.line(image, pt1, pt2, color, thickness)
    # Start: (640-20, 360) = (620, 360)   Left arm
    # End:   (640+20, 360) = (660, 360)   Right arm
    # Color: (0,255,0) = Pure GREEN
    # Thickness: 1px

# Vertical line: up To down
    # Start: (640, 360-20) = (640, 340)   Top arm
    # End:   (640, 360+20) = (640, 380)   Bottom arm

# ======================================
# Telemetry Data on screen:

        # Iteration 1: key='alt', value=125.5
        # text = "ALT: 125.5"
        # cv2.putText(..., (10, 20))  → Line 1 at y=20px

        # Iteration 2: key='speed', value=15.2  
        # text = "SPD: 15.2"
        # cv2.putText(..., (10, 40))  → Line 2 at y=40px

        # Iteration 3: key='heading', value=270
        # text = "HDG: 270"
        # cv2.putText(..., (10, 60))  → Line 3 at y=60px

        # Iteration 4: key='battery', value=85
        # text = "BAT: 85"
        # cv2.putText(..., (10, 80))  → Line 4 at y=80px

