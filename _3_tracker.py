# tracker.py
import numpy as np

class TargetSelector:
    def __init__(self, model):
        self.model = model
        self.vehicle_classes = ['car', 'motorcycle', 'bus', 'truck']
        self.locked_id = None

    def select_target(self, results, frame_shape):
        h, w, _ = frame_shape
        center = np.array([w/2, h/2])  # suppose pixel is 1280x720 divide with 2 -> 640,360

        min_dist = float("inf")
        selected_box = None

        for r in results:
            for box in r.boxes:
                if box.id is None:
                    continue

                cls = self.model.names[int(box.cls[0])]  # class ID
                if cls not in self.vehicle_classes:  # car, motorcycle, bus, truck
                    continue                 # Skip non-vehicles

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy() 
                box_center = np.array([(x1+x2)/2, (y1+y2)/2]) # Box (x1=100,y1=200,x2=300,y2=400) → box_center = [200, 300]

                dist = np.linalg.norm(center - box_center)  # Euclidean distance formula √((640-200)² + (360-300)²) = 448 pixels

                if dist < min_dist:
                    min_dist = dist
                    selected_box = box

        return selected_box
    
    

# ----------------Example for closest to center---------------

# min_dist = float("inf")    -->  Start: infinit
# selected_box = None

# First vehicle (dist = 450 pixels)
# dist = 450
# if 450 < ∞:                  # TRUE
#     min_dist = 450           # Update: 450
#     selected_box = box1

# Second vehicle (dist = 200 pixels)  
# dist = 200
# if 200 < 450:                # TRUE
#     min_dist = 200           # Update: 200
#     selected_box = box2

# Third vehicle (dist = 600 pixels)
# dist = 600
# if 600 < 200:                # FALSE
    # Skip - not closer
