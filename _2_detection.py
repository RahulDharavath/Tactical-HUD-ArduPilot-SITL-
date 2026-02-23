from ultralytics import YOLO

class Detector:  # reusable class
    def __init__(self, model_path="yolov8n.pt"):  
        self.model = YOLO(model_path)  # load pre trained model 
        self.vehicle_classes = ['car', 'motorcycle', 'bus', 'truck']

    def detect(self, frame):
        results = self.model.track(frame,          # Run YOLO tracking
                                   persist=True,   # Keep track IDs across frames
                                   tracker="bytetrack.yaml")
        return results
