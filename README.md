# Tactical-HUD-ArduPilot-SITL-

---

## Overview

This project implements a real-time Tactical Heads-Up Display (HUD) system that integrates:
* Live telemetry from ArduPilot SITL
* Real-time vehicle detection
* Persistent object tracking
* Automatic target locking
* Tactical-style HUD overlay
* Performance optimization for real-time execution

The goal is to simulate a surveillance-style system where a drone streams telemetry and video, detects vehicles, and automatically locks onto a target.

---

In this project we are using simulation software instead of using real drone hardware,and they are  three software compontent using it.

  1. For Drone --> We are using ArduPilot + SITL. Which acts as real drone.
  2. For Ground Control Station (GCS) --> we are using MAVProxy.
  3. For communication --> we are using MAVLink.

## ArduPilot:
ArduPilot is the world's most advanced & versatile open-source autopilot software system. It is designed to provide autonomus control, stablization, and mission capabilities for a vast range of unmanned vehicles.

## SITL: Software In The Loop
SITL is a simulation method that allow us to run autopilot firmware(like Ardupilot or PX4) directly on our computer processor without needing any physical flight controller hardware.

## MAVProxy:
MAVProxy is a lightweight, terminal-based ground control station (GCS) for MAVLink-enabled drones like ArduPilot SITL. It provides real-time telemetry monitoring and direct flight control through simple text commands.

            # Flight control
    mode GUIDED/AUTO     # Enable guided/autonomous mode
    arm throttle         # Arm virtual motors  
    takeoff 30           # Takeoff to 30m altitude
    wp 100 100 50        # Fly to waypoint (lat,lon,alt)
    land                 # Auto-land
    
          # Status monitoring
    status               # Full telemetry dump
    param show BAT*      # Battery parameters
    rc 3 1500            # Manual RC override

## MAVLink:
MAVLink stands for Micro Air Vehicle Link.

It is a lightweight, open-source messaging protocol designed for communication between drones (UAVs) and ground control systems.
It sends small structured packets.

Each packet contains:
* Message ID
* Data fields (sensor data like GPS, speed, battery, etc.)
* Timestamp

These structured packets allow different systems to understand drone data in a standardized format.

## What is Telemetry?
Telemetry is a system that sends its internal status data to another system in real time.

Example: A car sends engine data to a service center.

In the same way, a drone sends its status data such as:
* Latitude
* Longitude
* Altitude
* Ground speed
* Mode
* Battery level
* 
Without telemetry, we cannot know the drone’s condition — like how much battery is left, which direction it is moving, or which flight mode it is using.
Telemetry is also used in different fields such as space systems, UAVs, automotive systems, etc.

## How Telemetry Works (Step-by-Step)

STEP 1: Sensor Data Collection
First, the drone collects data from its sensors, such as:
* GPS (for location)
* Battery sensor (battery status)
* Acceleration 
* Rotation, etc

STEP 2: Autopilot Processes the Data

Software like  ArduPilot acts as the drone’s brain.

It continuously processes:
* GPS position
* Sensor data
* IMU readings
* Battery information.

It converts raw sensor data into meaningful flight information like:
* Current latitude and longitude
* Current altitude
* Current speed
* Current flight mode

STEP 3: MAVLink Message Creation

After processing the data, it is packed into structured messages using a protocol called MAVLink.

It sends small structured packets.

Each packet contains:
* Message ID
* Data fields (sensor data like GPS, speed, battery, etc.)
* Timestamp
These structured packets allow different systems to understand drone data in a standardized format.

STEP 4: Message Transmission
After MAVLink creates the message packets, they must be transmitted from the drone to another system.
This transmission can happen through:
* Radio (real drone communication)
* Serial connection
* UDP
* TCP
  
UDP is simply a communication method that allows two programs to send data to each other over a network connection.

To Decode the mavlink data message we are using pymavlink (python library.

---

## ArduPilot SITL Installation:

STEP 1: Install Homebrew

    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    
STEP 2: System Package

    brew install git
    brew install wget
    brew install gawk
    brew install coreutils

STEP 3: Python Package

    python3.9
    pymavlink
    MAVProxy
    opencv-python==4.10.0.84
    ultralytics
    numpy==1.26.4

STEP 4: Clone ArduPilot Repository

    mkdir ArduPilot_SITL
    cd ~/Desktop/ArduPilot_SITL
    git clone --recursive https://github.com/ArduPilot/ardupilot.git
    cd ardupilot

STEP 5: Update Submodule

    # This step is REQUIRED for SITL to work
    git submodule update --init --recursive

STEP 6: Install Prerequisites Script

    # Run Mac-specific installer
    Tools/environment_install/install-prereqs-mac.sh -y

STEP 7: Configure SITL Build

    ./waf configure --board sitl

STEP 8: Build ArduCopter

    # Build Copter firmware for SITL
    ./waf copter

STEP 9: Run SITL

    Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

Workflow:

Terminal 1
    
    cd ~/Desktop/ArudPilot_SITL/ardupilot
    Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
    
Terminal 2 

    cd ~/Desktop/ArudPilot_SITL
    source myvenv/bin/activate
    python3 main.py

---

## System Architecture
<img width="824" height="688" alt="Screenshot 2026-02-23 at 2 53 02 PM" src="https://github.com/user-attachments/assets/ea25fc02-7321-4c4a-b35a-336f9a7bf641" />

---
## Telemetry Module:
The TelemetryHandler module connects to ArduPilot SITL using MAVLink protocol & extracts real time telemetry data for HUD display.

    from pymavlink import mavutil
  This library allows communication with flight controllers or SITL.
  
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
  127.0.0.1 → Local machine | 14550 → Default MAVLink UDP port from SITL

    self.conn = mavutil.mavlink_connection(connection_string)
    self.conn.wait_heartbeat()
  It creates MAVLink connection. and Waits until a heartbeat message is received.

  Without 'wait_heartbeat()', telemetry reading might start before connection is ready.

    self.data = {
    "lat": None,
    "lon": None,
    "alt": None,
    "ground_speed": None,
    "heading": None,
    "mode": None,
    "battery": None
    }
  It stores latest telemetry data. and easy to pass into HUD display.

    msg = self.conn.recv_match(blocking=False)
  Why we using blocking=False bcz it prevents freezing video loop. &  even if no telemetry message available -> immediately continue.
  Its Ensures smooth real-time performance.

    msg_type = msg.get_type()
    if msg_type == "GLOBAL_POSITION_INT":
    elif msg_type == "VFR_HUD":
    elif msg_type == "SYS_STATUS":
    elif msg_type == "HEARTBEAT":
  Every MAVLink message has a type name.
  which provides:
  * Latitude, * Longitude, * Relative altitude, * Ground speed, * Heading, * battery life, * mode (like guided, auto, stablize)

---

## Tracker Module:
I used Euclidean distance formula to track closer to center crossline.

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

---

## HUD Module:

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

---
## Main Module:

    telemetry = TelemetryHandler()
    detector = Detector()
    selector = TargetSelector(detector.model)
    hud = HUDRenderer()
* Initializes all modules.
* Telemetry connects to SITL via MAVLink.
* Detector loads YOLO model.
* Tracker handles automatic vehicle locking.
* HUD module handles all overlay drawing.

      cap = cv2.VideoCapture("ss.mp4")
* Opens video file.
* Acts as simulated UAV camera feed.
* Frames are processed in real time.

      frame_count = 0
      skip_frames = 2
      last_results = None
      fps = 0
  skip_frames reduces detection frequency.
Detection runs every 2 frames.
Improves performance while keeping tracking stable.
fps stores smoothed FPS value.

      while True:
  Each loop processes one video frame.

Step 1: Read Frame

      ret, frame = cap.read()
  * Reads next frame from video.
  * If no frame available → loop stops.

Step 2: Resize Frame

      frame = cv2.resize(frame, (416, 416))

* Reduces resolution.
* Speeds up YOLO inference.
* Improves overall FPS.

Step 3: Detection with Skip Logic

      if frame_count % skip_frames == 0:
      results = detector.detect(frame)
* Detection runs every 2 frames.
* Other frames reuse previous results.
* Reduces CPU load.
* Maintains smooth visual tracking.

Step 4: Draw Detection Boxes

For each detected object:
* Extract bounding box coordinates
* Get confidence score
* Get class label (car, bus, truck, etc.)
* Get tracking ID from ByteTrack
* Draw box and label on screen
* Ex: ID:3 car 0.87
  
Step 5: Target Selection

    target_box = selector.select_target(results, frame.shape)
* Automatically selects vehicle closest to screen center.
* Implements auto-lock logic.
* Returns selected bounding box.

Step 6: Telemetry Update

      telem_data = telemetry.update()
      
* Fetches live telemetry from SITL.
* Non-blocking (does not slow video).

Returns:
* Latitude
* Longitude
* Altitude
* Speed
* Heading
* Mode
* Battery

Step 7: HUD Rendering

    hud.draw_crosshair(annotated)
    hud.draw_telemetry(annotated, telem_data)
    hud.draw_timestamp(annotated)
    hud.highlight_target(annotated, target_box)

Draws tactical interface elements:
* Center crosshair
* Telemetry text
* Timestamp
* Highlight locked target

Step 8: FPS Calculation

    new_fps = 1 / (time.time() - start)
    fps = 0.9 * fps + 0.1 * new_fps
* Measures time taken per frame.
* Calculates frames per second.
* Applies smoothing for stable display.
* Shows performance on screen.

Step 9: Display Output

    cv2.imshow("AI Tactical HUD", annotated)

  Displays final output combining:
* Detection
* Tracking
* Telemetry
* HUD
* FPS

step 10: Clean Exit

    cap.release()
    cv2.destroyAllWindows()
* Releases video resources.
* Prevents memory leaks.
* Safely closes program.

---
## video





https://github.com/user-attachments/assets/e9de7181-b84b-4909-9224-2af2fafa469f




