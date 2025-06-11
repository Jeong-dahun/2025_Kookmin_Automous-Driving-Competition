#🏎️ 2025_Kookmin_Autonomous-Driving-Competition
![슬라이드2](https://github.com/user-attachments/assets/a6fda189-8e58-4c75-a03b-dbaf816cc113)

## 🛠️ Development Environment
- OS: Ubuntu 20.04 LTS
- ROS: ROS 1 Noetic 
- Language: Python 3.8

## 📁 Repository Structure 
<pre> 2025_Kookmin_Automous-Driving-Competition/ 
  └── src/ 
        └── kookmin/ 
                └── driver/ 
                      ├── april.py  
                      ├── cam_test.py  
                      ├── filter_pid.py  
                      ├── hough.py  
                      ├── lidar_test.py  
                      ├── lidar_viewer.py  
                      ├── track_drive.py  
                      ├── traffic.py  
                      └── ultra.py  </pre>

## 🎯 Main Tasks

### 🚦 Traffic Sign Detection 

This module detects a traffic light and initiates vehicle start only when the rightmost light is blue.

#### ✅ Key Features
- Defines a fixed **Region of Interest (ROI)** to focus on the traffic light area in the image.
- Uses **Hough Circle Transform** to detect exactly **three circular lights**.
- Verifies that:
  - The circles are **horizontally aligned** (low Y-axis variation).
  - The circles have **sufficient and balanced X-axis spacing**.
- Computes **average brightness** for each circle and highlights the brightest one.
- If the **brightest circle is on the rightmost position**, it is considered **blue**, and the system returns `True`.



### 🛣️  Lane Drive

This module detects left and right lane lines from a monocular camera image and calculates their intersection points to derive a midpoint for steering control.

#### ✅ Key Features
- Defines a fixed **Region of Interest (ROI)** to focus on the road.
- Applies standard image preprocessing:
  - Grayscale conversion  
  - Gaussian blur  
  - Canny edge detection
- Uses a trapezoidal mask to isolate lane-relevant area.
- Applies **Hough Line Transform** to extract line segments.
- Filters out horizontal lines by slope thresholding.
- Separates line segments into **left and right lanes** based on slope and position.
- Fits representative lines to each side using slope-intercept averaging.
- Calculates:
  - Left and right lane positions at a reference row
  - Midpoint between lanes
  - Offset from the image center for steering control

### 🟠 SENSOR DRIVE (Cone Driving)

This mode enables autonomous driving through cones using LiDAR data only.

#### ✅ Key Features
- Activated when cones are detected ahead (based on front LiDAR distance < 5m).
- Uses **median filtering** to reduce LiDAR noise.
- Focuses on **left (210°–355°)** and **right (5°–150°)** angular sectors.
- Implements a **gap-finding algorithm**:
  - Detects contiguous "free space" areas larger than 2.5m.
  - Selects the widest gap and calculates its midpoint bearing.
  - Converts the bearing to a steering angle.
- Applies different PID gains depending on whether free space is detected on both sides or only one.
- Returns a clipped steering angle between **-75° and +75°**.

#### 🔁 Driving Flow
1. Monitor front LiDAR distance.
2. When cones are detected:
   - `lidar_drive()` → `steer_through_cones()` is executed.
   - Vehicle adjusts angle and slows down to speed = 8.
3. Continues counting successful frames (`cone_clear_count`).
4. After 420 frames (~42 seconds), mode switches back to lane driving (`LANE_DRIVE`).

#### 📌 Steering Logic (Simplified)
```python
if left side only free:
    steer toward left gap
elif right side only free:
    steer toward right gap
else:
    steer toward center of widest gap
```

### 🚗💨 OBJECT DRIVE (Obstacle Avoidance)

This mode enables **dynamic lane changing and obstacle avoidance** when a slower vehicle is detected in front, and faster vehicles are occupying adjacent lanes.

#### ✅ Task Overview
- The ego vehicle detects a slow-moving car in its lane.
- Neighboring lanes also contain vehicles, but they are moving faster and will eventually stop.
- The ego vehicle must opportunistically change lanes **before the front vehicle stops**, pass it, and return to the original lane.
- This is a **time-sensitive overtaking task** that must be executed safely using LiDAR-based logic and simple staged control.

#### 🧠 Implementation Summary
The `OBJECT_DRIVE` mode is structured into **4 stages**, managed by a state machine:

| Stage | Description |
|-------|-------------|
| **0** | Wait until front vehicle is close (based on LiDAR percentile distance) |
| **1** | Fixed steering angle for ~2.5 seconds to perform initial lane change |
| **2** | Follow the new lane using PID lane tracking |
| **3** | Once another vehicle is detected again in new lane, steer back to original lane |

#### ⚙️ Key Logic
- **Trigger condition**: `front < 10.0` (LiDAR-based) while in `LANE_DRIVE`
- **Transition to OBJECT_DRIVE**: Sets `overtake_stage = 0`
- **Steering**: Fixed values during lane shift (e.g., -20° to left, then +20° to return)
- **Speed**: Dynamically adjusted depending on steering angle
- **Safety buffer**: 0.5s delay before allowing re-entry (to avoid oscillation)

#### 📌 Code Highlights
```python
if self.overtake_stage == 0 and front < TH:
    self.overtake_stage = 1
    self.overtake_start_time = now

elif self.overtake_stage == 1:
    if elapsed < 2.5:
        self.drive(-20, 40)  # Initial lane change
    else:
        self.overtake_stage = 2

elif self.overtake_stage == 2:
    # PID lane following in new lane
    if front < 11.0:
        self.overtake_stage = 3

elif self.overtake_stage == 3:
    if elapsed < 2.5:
        self.drive(+20, 35)  # Return to original lane
    else:
        drive_mode = LANE_DRIVE
```
