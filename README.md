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



### 2. Lane Drive

### 3. SENSOR DRIVE (Cone Driving)

### 4. OBJECT DRIVE (Obstacle Avoidance)
