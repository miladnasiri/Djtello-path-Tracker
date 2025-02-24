import numpy as np
from slam_system import SLAMSystem
from controller import AdvancedController
from visualization import Visualizer
from enhanced_visualization import EnhancedVisualizer
import time
import pandas as pd

class DigitalTwin:
    def __init__(self):
        # Initialize subsystems
        self.slam = SLAMSystem()
        self.controller = AdvancedController()
        self.visualizer = EnhancedVisualizer()
        
        # Initialize state variables
        self.state = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'attitude': np.zeros(3),
            'angular_velocity': np.zeros(3)
        }
        
        # Data logging
        self.flight_data = []
        
    def update(self, sensor_data):
        """Update digital twin state"""
        # Update SLAM
        pose = self.slam.process_frame(sensor_data['image'], sensor_data['imu'])
        
        # Update state estimation
        self.update_state(pose, sensor_data)
        
        # Update visualization
        self.visualizer.update(self.state)
        
        # Log data
        self.log_data()
        
    def update_state(self, pose, sensor_data):
        """Update state estimation"""
        # Update position and attitude from SLAM
        self.state['position'] = pose[:3, 3]
        self.state['attitude'] = self.euler_from_matrix(pose[:3, :3])
        
        # Update velocities from IMU
        dt = sensor_data['dt']
        self.state['velocity'] += sensor_data['imu']['acceleration'] * dt
        self.state['angular_velocity'] = sensor_data['imu']['gyro']
        
    def compute_control(self, target_state):
        """Compute control commands"""
        return self.controller.compute_control(self.state, target_state)
        
    def log_data(self):
        """Log current state and time"""
        self.flight_data.append({
            'timestamp': time.time(),
            'x': self.state['position'][0],
            'y': self.state['position'][1],
            'z': self.state['position'][2],
            'roll': self.state['attitude'][0],
            'pitch': self.state['attitude'][1],
            'yaw': self.state['attitude'][2]
        })
        
    def save_data(self, filename='flight_data.csv'):
        """Save logged data to CSV"""
        df = pd.DataFrame(self.flight_data)
        df.to_csv(filename, index=False)
        
    @staticmethod
    def euler_from_matrix(R):
        """Convert rotation matrix to euler angles"""
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
            
        return np.array([x, y, z])

if __name__ == "__main__":
    # Example usage
    twin = DigitalTwin()
    
    # Simulate some sensor data
    sensor_data = {
        'image': np.zeros((480, 640, 3), dtype=np.uint8),
        'imu': {
            'acceleration': np.zeros(3),
            'gyro': np.zeros(3)
        },
        'dt': 0.1
    }
    
    # Run simulation
    try:
        for _ in range(100):  # Simulate 10 seconds
            twin.update(sensor_data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        twin.save_data()
