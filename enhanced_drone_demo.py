import numpy as np
import cv2
import time
from digital_twin import DigitalTwin
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math

class EnhancedDroneDemo:
    def __init__(self):
        self.twin = DigitalTwin()
        self.current_position = np.zeros(3)
        self.target_position = np.array([2.0, 2.0, 1.0])  # Default target
        self.path_history = []
        self.planned_path = []
        
        # Setup interactive plot
        self.setup_interactive_plot()
        
    def setup_interactive_plot(self):
        """Setup interactive plot with target selection"""
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        
        # Main 3D plot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_title('3D View with Target')
        
        # Top view for clicking targets
        self.ax_top = self.fig.add_subplot(222)
        self.ax_top.set_title('Top View - Click to Set Target')
        
        # Plot for metrics
        self.ax_metrics = self.fig.add_subplot(223)
        self.ax_metrics.set_title('Distance to Target')
        
        # Status and info
        self.ax_info = self.fig.add_subplot(224)
        self.ax_info.set_axis_off()
        
        # Set interaction
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Add control buttons
        self.add_buttons()
        
        plt.tight_layout()
        
    def add_buttons(self):
        """Add control buttons"""
        self.btn_ax_start = plt.axes([0.81, 0.05, 0.1, 0.04])
        self.btn_start = Button(self.btn_ax_start, 'Start Mission')
        self.btn_start.on_clicked(self.start_mission)
        
        self.btn_ax_stop = plt.axes([0.68, 0.05, 0.1, 0.04])
        self.btn_stop = Button(self.btn_ax_stop, 'Stop')
        self.btn_stop.on_clicked(self.stop_mission)
        
    def on_click(self, event):
        """Handle click events for target selection"""
        if event.inaxes == self.ax_top:
            self.target_position = np.array([event.xdata, event.ydata, 1.0])
            self.update_plots()
            print(f"New target set: {self.target_position}")
            
    def generate_planned_path(self):
        """Generate a planned path to target"""
        start = self.current_position
        end = self.target_position
        num_points = 50
        
        # Create a smooth path
        t = np.linspace(0, 1, num_points)
        path = np.zeros((num_points, 3))
        
        # Add some curvature to the path
        for i in range(num_points):
            path[i] = (1-t[i])*start + t[i]*end
            path[i, 2] = start[2] + 2*t[i]*(1-t[i])  # Parabolic height profile
            
        self.planned_path = path
            
    def update_plots(self):
        """Update all plots"""
        # Clear plots
        self.ax_3d.cla()
        self.ax_top.cla()
        self.ax_metrics.cla()
        
        # Plot current position and history
        if self.path_history:
            path = np.array(self.path_history)
            self.ax_3d.plot3D(path[:, 0], path[:, 1], path[:, 2], 'b-', label='Actual Path')
            self.ax_top.plot(path[:, 0], path[:, 1], 'b-')
        
        # Plot planned path
        if len(self.planned_path) > 0:
            self.ax_3d.plot3D(self.planned_path[:, 0], 
                            self.planned_path[:, 1], 
                            self.planned_path[:, 2], 
                            'g--', label='Planned Path')
            self.ax_top.plot(self.planned_path[:, 0], 
                           self.planned_path[:, 1], 'g--')
        
        # Plot current position
        self.ax_3d.scatter([self.current_position[0]], 
                         [self.current_position[1]], 
                         [self.current_position[2]], 
                         color='blue', s=100, label='Drone')
        
        # Plot target
        self.ax_3d.scatter([self.target_position[0]], 
                         [self.target_position[1]], 
                         [self.target_position[2]], 
                         color='red', s=100, label='Target')
        
        # Top view
        self.ax_top.scatter([self.current_position[0]], 
                          [self.current_position[1]], 
                          color='blue', s=100)
        self.ax_top.scatter([self.target_position[0]], 
                          [self.target_position[1]], 
                          color='red', s=100)
        
        # Set limits and labels
        for ax in [self.ax_3d, self.ax_top]:
            ax.set_xlim([-5, 5])
            ax.set_ylim([-5, 5])
            if ax == self.ax_3d:
                ax.set_zlim([0, 5])
                
        # Plot distance metric
        if self.path_history:
            distances = [np.linalg.norm(p - self.target_position) 
                       for p in self.path_history]
            self.ax_metrics.plot(distances, 'b-')
            self.ax_metrics.set_ylabel('Distance to Target (m)')
            self.ax_metrics.set_xlabel('Time Step')
            
        # Update info text
        self.update_info_text()
        
        self.ax_3d.legend()
        plt.draw()
        plt.pause(0.01)
        
    def update_info_text(self):
        """Update status and info text"""
        self.ax_info.clear()
        self.ax_info.set_axis_off()
        
        info_text = f"""
        Current Position: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}]
        Target Position: [{self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f}]
        Distance to Target: {np.linalg.norm(self.current_position - self.target_position):.2f} m
        """
        self.ax_info.text(0.1, 0.5, info_text, fontsize=10)
        
    def start_mission(self, event):
        """Start the drone mission"""
        self.running = True
        self.generate_planned_path()
        self.run_simulation()
        
    def stop_mission(self, event):
        """Stop the drone mission"""
        self.running = False
        
    def run_simulation(self):
        """Run the main simulation loop"""
        print("Starting mission - Click 'Stop' to end")
        
        while self.running:
            # Update digital twin
            sensor_data = self.generate_sensor_data()
            self.twin.update(sensor_data)
            
            # Update current position
            self.current_position = self.twin.state['position']
            self.path_history.append(self.current_position.copy())
            
            # Update visualization
            self.update_plots()
            
            # Check if reached target
            if np.linalg.norm(self.current_position - self.target_position) < 0.1:
                print("\nTarget reached!")
                self.running = False
                break
                
            time.sleep(0.1)
    
    def generate_sensor_data(self):
        """Generate simulated sensor data"""
        # Calculate direction to target
        direction = self.target_position - self.current_position
        distance = np.linalg.norm(direction)
        if distance > 0:
            direction = direction / distance
            
        # Create sensor data
        sensor_data = {
            'image': np.zeros((480, 640, 3), dtype=np.uint8),
            'imu': {
                'acceleration': direction * min(distance, 0.5),
                'gyro': np.zeros(3),
                'dt': 0.1
            },
            'dt': 0.1
        }
        
        return sensor_data

if __name__ == "__main__":
    demo = EnhancedDroneDemo()
    plt.show(block=True)  # Keep the window open
