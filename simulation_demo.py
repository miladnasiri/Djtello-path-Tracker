import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class DroneSimulator:
    def __init__(self):
        self.position = np.array([0., 0., 0.])
        self.velocity = np.array([0., 0., 0.])
        self.target = np.array([2., 2., 1.])
        self.trajectory = []
        
        # Setup visualization
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        self.setup_plots()
        
    def setup_plots(self):
        # 3D trajectory
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_title('3D Drone Path')
        
        # Top view
        self.ax_top = self.fig.add_subplot(222)
        self.ax_top.set_title('Top View (XY)')
        
        # Camera view
        self.ax_camera = self.fig.add_subplot(223)
        self.ax_camera.set_title('Simulated Camera')
        
        # Status
        self.ax_status = self.fig.add_subplot(224)
        self.ax_status.set_title('Flight Status')
        
        plt.tight_layout()
        
    def generate_camera_view(self):
        """Generate simulated camera view with features"""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add grid pattern
        grid_size = 50
        for i in range(0, 640, grid_size):
            cv2.line(img, (i, 0), (i, 480), (50, 50, 50), 1)
        for i in range(0, 480, grid_size):
            cv2.line(img, (0, i), (640, i), (50, 50, 50), 1)
            
        # Add some feature points
        for _ in range(20):
            x = np.random.randint(0, 640)
            y = np.random.randint(0, 480)
            cv2.circle(img, (x, y), 3, (0, 255, 0), -1)
            
        return img
        
    def update(self):
        # Update position based on simple dynamics
        direction = self.target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:
            # Move towards target
            self.velocity = direction * 0.1
            self.position += self.velocity
            
        # Store trajectory
        self.trajectory.append(self.position.copy())
        
        # Update visualization
        self.update_plots()
        
    def update_plots(self):
        # Clear plots
        self.ax_3d.cla()
        self.ax_top.cla()
        self.ax_status.cla()
        
        # Get trajectory data
        if self.trajectory:
            trajectory = np.array(self.trajectory)
            
            # 3D plot
            self.ax_3d.plot3D(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'b-')
            self.ax_3d.scatter([self.position[0]], [self.position[1]], [self.position[2]], 
                             color='blue', s=100, label='Drone')
            self.ax_3d.scatter([self.target[0]], [self.target[1]], [self.target[2]], 
                             color='red', s=100, label='Target')
            
            # Set consistent view
            self.ax_3d.set_xlim([-3, 3])
            self.ax_3d.set_ylim([-3, 3])
            self.ax_3d.set_zlim([0, 3])
            self.ax_3d.set_title('3D Drone Path')
            self.ax_3d.legend()
            
            # Top view
            self.ax_top.plot(trajectory[:, 0], trajectory[:, 1], 'b-')
            self.ax_top.scatter([self.position[0]], [self.position[1]], 
                              color='blue', s=100, label='Drone')
            self.ax_top.scatter([self.target[0]], [self.target[1]], 
                              color='red', s=100, label='Target')
            self.ax_top.set_xlim([-3, 3])
            self.ax_top.set_ylim([-3, 3])
            self.ax_top.set_title('Top View (XY)')
            self.ax_top.grid(True)
            self.ax_top.legend()
            
        # Camera view
        camera_img = self.generate_camera_view()
        self.ax_camera.imshow(camera_img)
        self.ax_camera.set_title('Simulated Camera View')
        
        # Status display
        status_text = f"""
        Position: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]
        Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f}]
        Distance to Target: {np.linalg.norm(self.target - self.position):.2f}
        """
        self.ax_status.text(0.1, 0.5, status_text, fontsize=10)
        self.ax_status.set_axis_off()
        
        plt.draw()
        plt.pause(0.01)

def run_simulation():
    print("Starting Drone Simulation...")
    print("This will show you what to expect when you have a real drone")
    print("Press Ctrl+C to stop")
    
    sim = DroneSimulator()
    
    try:
        while True:
            sim.update()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")

if __name__ == "__main__":
    run_simulation()
