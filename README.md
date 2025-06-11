# 🏎️ 2025_Kookmin_Autonomous-Driving-Competition
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
    steer toward center of widest gap ```

### 4. OBJECT DRIVE (Obstacle Avoidance)
