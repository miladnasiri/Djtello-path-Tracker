import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d

class Visualizer:
    def __init__(self):
        # Setup matplotlib figure
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)  # Top view
        self.ax_xz = self.fig.add_subplot(223)  # Front view
        self.ax_yz = self.fig.add_subplot(224)  # Side view
        
        # Initialize trajectory history
        self.trajectory = []
        
        # Setup Open3D visualizer for point cloud
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        
        # Create coordinate frame
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0])
        self.vis.add_geometry(self.coord_frame)
        
        # Create drone mesh
        self.drone_mesh = self._create_drone_mesh()
        self.vis.add_geometry(self.drone_mesh)
        
    def _create_drone_mesh(self):
        """Create a simple drone mesh for visualization"""
        # Create a simple box to represent the drone
        drone = o3d.geometry.TriangleMesh.create_box(
            width=0.2,   # x dimension
            height=0.2,  # y dimension
            depth=0.05   # z dimension
        )
        
        # Move drone center to origin
        drone.translate([-0.1, -0.1, -0.025])
        
        # Add color to the drone (red)
        drone.paint_uniform_color([1, 0, 0])
        
        return drone
        
    def update(self, state):
        """Update visualization with new state"""
        # Update trajectory
        position = state['position']
        self.trajectory.append(position)
        
        # Update plots
        self._update_trajectory_plots()
        
        # Update 3D visualization
        self._update_3d_visualization(state)
        
    def _update_trajectory_plots(self):
        """Update all trajectory plots"""
        # Convert trajectory to numpy array
        if len(self.trajectory) > 0:
            trajectory = np.array(self.trajectory)
            
            # Clear all axes
            self.ax_3d.cla()
            self.ax_xy.cla()
            self.ax_xz.cla()
            self.ax_yz.cla()
            
            # Plot 3D trajectory
            self.ax_3d.plot3D(
                trajectory[:, 0],
                trajectory[:, 1],
                trajectory[:, 2],
                'b-'
            )
            self.ax_3d.set_title('3D Trajectory')
            self.ax_3d.set_xlabel('X')
            self.ax_3d.set_ylabel('Y')
            self.ax_3d.set_zlabel('Z')
            
            # Plot top view (XY plane)
            self.ax_xy.plot(trajectory[:, 0], trajectory[:, 1], 'b-')
            self.ax_xy.set_title('Top View')
            self.ax_xy.set_xlabel('X')
            self.ax_xy.set_ylabel('Y')
            
            # Plot front view (XZ plane)
            self.ax_xz.plot(trajectory[:, 0], trajectory[:, 2], 'b-')
            self.ax_xz.set_title('Front View')
            self.ax_xz.set_xlabel('X')
            self.ax_xz.set_ylabel('Z')
            
            # Plot side view (YZ plane)
            self.ax_yz.plot(trajectory[:, 1], trajectory[:, 2], 'b-')
            self.ax_yz.set_title('Side View')
            self.ax_yz.set_xlabel('Y')
            self.ax_yz.set_ylabel('Z')
            
            plt.tight_layout()
            plt.draw()
            plt.pause(0.001)
            
    def _update_3d_visualization(self, state):
        """Update Open3D visualization"""
        # Update drone position
        position = state['position']
        attitude = state['attitude']
        
        # Create transformation matrix
        T = np.eye(4)
        T[:3, 3] = position
        
        # Update drone mesh transform
        self.drone_mesh.transform(T)
        
        # Update visualization
        self.vis.update_geometry(self.drone_mesh)
        self.vis.poll_events()
        self.vis.update_renderer()
