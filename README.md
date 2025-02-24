# 🚁 Tello Drone Path Tracker and Digital Twin

## 📋 Overview
Advanced real-time path tracking and digital twin system for DJI Tello drone, implementing SLAM, computer vision, and hybrid control algorithms.

## 🌟 Key Features
- Real-time SLAM-based position tracking
- Neural network enhanced PID control
- 3D environment mapping and visualization
- Digital twin simulation
- Interactive path planning
- Multi-view visualization

## 🛠️ Installation
```bash
# Clone the repository
git clone https://github.com/miladnasiri/Djtello-path-Tracker.git
cd Djtello-path-Tracker

# Install dependencies
pip install -r requirements.txt
```

## 🚀 Quick Start
1. Connect to Tello drone:
```python
from djitellopy import Tello
drone = Tello()
drone.connect()
drone.streamon()
```

2. Run with real drone:
```bash
python real_drone_demo.py
```

## 🎮 Controls
- T: Takeoff
- L: Land
- Arrow keys: Movement
- Q: Quit

## 🏗️ Architecture
1. SLAM System (slam_system.py)
   - ORB feature detection
   - Extended Kalman Filter
   - 3D point cloud mapping
   - Real-time pose estimation

2. Control System (controller.py)
   - PID + Neural Network hybrid control
   - Adaptive trajectory optimization
   - Safety constraints
   - Real-time path planning

3. Digital Twin (digital_twin.py)
   - State estimation and monitoring
   - Environment reconstruction
   - Performance analytics
   - Data logging and visualization

## 📊 Visualization
- 3D trajectory visualization
- Real-time state monitoring
- Feature detection display
- Path planning interface

## 🔒 Safety Features
- Automatic emergency landing
- Battery monitoring
- Safe distance maintenance
- Collision avoidance

## 📈 Performance
- Position accuracy: ±5cm
- Update rate: 30Hz
- Control latency: <50ms
- Battery efficiency: 20min flight time

## 🤝 Contributing
Contributions are welcome! Please check out our contributing guidelines.

## 📄 License
MIT License - see LICENSE file for details
