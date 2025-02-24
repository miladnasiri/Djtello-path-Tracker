import numpy as np
import cv2
import time
from digital_twin import DigitalTwin
import math

class SimulatedSensorData:
    def __init__(self):
        self.t = 0
        self.dt = 0.1
        self.image_size = (480, 640)
        
    def generate_figure8_trajectory(self):
        """Generate figure-8 trajectory data"""
        scale = 2.0
        frequency = 0.2  # Hz
        omega = 2 * np.pi * frequency
        
        # Position
        x = scale * np.sin(omega * self.t)
        y = scale * np.sin(omega * self.t * 2) / 2
        z = 1.0 + 0.2 * np.sin(omega * self.t * 3)
        
        # Velocity
        vx = scale * omega * np.cos(omega * self.t)
        vy = scale * omega * np.cos(omega * self.t * 2)
        vz = 0.6 * omega * np.cos(omega * self.t * 3)
        
        # Create synthetic image with features
        image = np.zeros((self.image_size[0], self.image_size[1], 3), dtype=np.uint8)
        for _ in range(50):
            px = np.random.randint(0, self.image_size[1])
            py = np.random.randint(0, self.image_size[0])
            cv2.circle(image, (px, py), 3, (255, 255, 255), -1)
        
        # Simulate IMU data
        imu_data = {
            'acceleration': np.array([vx, vy, vz]) / self.dt,
            'gyro': np.array([0.1 * np.sin(omega * self.t), 
                            0.1 * np.cos(omega * self.t), 
                            0.2 * np.sin(omega * self.t * 2)]),
            'dt': self.dt
        }
        
        # Package sensor data
        sensor_data = {
            'image': image,
            'imu': imu_data,
            'dt': self.dt
        }
        
        # Update time
        self.t += self.dt
        
        return sensor_data, np.array([x, y, z])

def run_demo():
    # Initialize digital twin and sensor simulator
    twin = DigitalTwin()
    sensor_sim = SimulatedSensorData()
    
    # Target state for the controller
    target_state = {
        'position': np.array([0.0, 0.0, 1.0]),
        'velocity': np.zeros(3),
        'attitude': np.zeros(3),
        'angular_velocity': np.zeros(3)
    }
    
    print("Starting Digital Twin Demo...")
    print("Press Ctrl+C to stop the simulation")
    
    try:
        while True:
            # Generate simulated sensor data
            sensor_data, true_position = sensor_sim.generate_figure8_trajectory()
            
            # Update digital twin
            twin.update(sensor_data)
            
            # Compute control commands
            control = twin.compute_control(target_state)
            
            # Print telemetry
            print(f"\rPosition: [{true_position[0]:.2f}, {true_position[1]:.2f}, {true_position[2]:.2f}] | "\
                  f"Control: [{control[0]:.2f}, {control[1]:.2f}, {control[2]:.2f}]", end="")
            
            # Simulate at 10Hz
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
        twin.save_data('demo_flight_data.csv')
        print("Flight data saved to demo_flight_data.csv")

if __name__ == "__main__":
    run_demo()
