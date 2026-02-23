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

In this project, we use simulation software instead of real drone hardware.

Three main software components are used:
  1. For Drone --> We are using ArduPilot + SITL. Which acts as real drone.
  2. For Ground Control Station (GCS) --> we are using MAVProxy.
  3. For communication --> we are using MAVLink.

## ArduPilot:
ArduPilot is the world's most advanced & versatile open-source autopilot software system. 

It provides autonomous control, stabilization, and mission capabilities for various unmanned vehicles such as drones, rovers, boats, and submarines.

## SITL: Software In The Loop
SITL is a simulation method that allow us to run autopilot firmware(like Ardupilot or PX4) directly on our computer processor without needing any physical flight controller hardware.

## MAVProxy:
MAVProxy is a lightweight, terminal-based ground control station (GCS) for MAVLink-enabled drones like ArduPilot SITL. 

It provides real-time telemetry monitoring and direct flight control through simple text commands.

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

It sends small structured data packets.

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
  
Without telemetry, we cannot know the drone’s condition — like how much battery is left, which direction it is moving, or which flight mode it is using.

Telemetry is also used in different fields such as space systems, UAVs, automotive systems, etc.

## How Telemetry Works

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

In this project, we use UDP for local communication.

UDP is simply a communication method that allows two programs to send data to each other over a network connection.

To decode MAVLink messages in Python, we use the pymavlink library.

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
    
    cd ~/Desktop/ArduPilot_SITL/ardupilot
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

`pymavlink` is used to establish communication with SITL.

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
  prevents the video loop from freezing and ensures smooth real-time performance.

    msg_type = msg.get_type()
    if msg_type == "GLOBAL_POSITION_INT":
    elif msg_type == "VFR_HUD":
    elif msg_type == "SYS_STATUS":
    elif msg_type == "HEARTBEAT":
  Every MAVLink message has a type name.
  
  The module extracts:
  
  * Latitude
  * Longitude
  * Relative altitude
  * Ground speed
  * Heading
  * battery level
  * flight mode (like guided, auto, stablize)

---

## Tracker Module: Target selection logic

The tracker selects the vehicle closest to the screen center using the Euclidean distance formula.

The logic works as follows:

* Start with infinite minimum distance.
* Compare each detected vehicle’s distance from the screen center.
* Update the selected vehicle if a closer one is found.
* Ignore vehicles that are farther away.
  
This ensures automatic and deterministic target locking.

    # Example:
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

The HUD module draws the tactical interface overlay.

It renders:

* A center crosshair
* Telemetry values (altitude, speed, heading, battery, etc.)
* Timestamp
* Highlighted locked target

The crosshair is drawn using horizontal and vertical lines centered in the frame.

Telemetry text is displayed line by line on the left side of the screen using `cv2.putText`.

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

The `main.py` file integrates all components together.

* Initializes all modules
* Opens video feed
* Runs detection and tracking
* Selects target
* Updates telemetry
* Draws the HUD
* Calculates FPS
* Displays final output

To improve performance, detection runs every two frames using skip logic. This reduces CPU usage while maintaining stable tracking.

FPS is calculated using:

    FPS = 1 / time_taken_per_frame
A smoothing formula is applied to prevent FPS fluctuation:

    fps = 0.9 * fps + 0.1 * new_fps

This ensures stable real-time performance display.

Clean Exit:


    cap.release()
    cv2.destroyAllWindows()
  Prevents memory leaks and ensures proper shutdown.
---

## Performance Results
Hardware Used

The system was tested on:

Machine: MacBook (CPU-based execution)
Processor: (1.8 GHz Dual-Core Intel Core i5)
RAM: (8 GB 1600 MHz DDR3)

---

## Model Configuration

Model: YOLOv8 Nano
Input Resolution: 416 × 416
Tracker: ByteTrack (built-in YOLO integration)
Frame Skip: 2 (detection runs every 2 frames)

The Nano model was selected to maintain a balance between detection accuracy and real-time performance.

---

## Optimization Techniques Applied

To meet real-time performance requirements, the following optimizations were implemented:

1. Frame Resizing
   
All frames are resized to 416×416 before inference.

Lower resolution reduces computational load and increases FPS.

2. Skip Logic
   
Detection runs every two frames instead of every frame.

Intermediate frames reuse previous detection results.

This reduces model inference load by approximately 50%.

3. Manual Bounding Box Rendering
   
Instead of using `results.plot()` (which is heavier), bounding boxes are drawn manually using OpenCV.

This reduces rendering overhead.

4. Non-Blocking Telemetry

This prevents the video loop from freezing and ensures smooth rendering.

5. FPS Smoothing
   
A weighted moving average is applied:
`fps = 0.9 * fps + 0.1 * new_fps`
This ensures stable FPS display and avoids jitter.


---
## video





https://github.com/user-attachments/assets/e9de7181-b84b-4909-9224-2af2fafa469f




