import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d

class EnhancedVisualizer:
    def __init__(self):
        plt.ion()  # Interactive mode
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_xz = self.fig.add_subplot(223)
        self.ax_yz = self.fig.add_subplot(224)
        
        # Initialize trajectory history
        self.trajectory = []
        self.max_trajectory_points = 1000
        
        # Setup Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        
        # Create coordinate frame and drone mesh
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0])
        self.drone_mesh = self._create_enhanced_drone_mesh()
        
        # Add geometries to visualizer
        self.vis.add_geometry(self.coord_frame)
        self.vis.add_geometry(self.drone_mesh)
        
        # Setup better lighting
        self._setup_visualization()
        
    def _create_enhanced_drone_mesh(self):
        """Create a more detailed drone mesh"""
        # Create main body
        body = o3d.geometry.TriangleMesh.create_box(width=0.3, height=0.3, depth=0.1)
        body.paint_uniform_color([1, 0, 0])  # Red
        
        # Create arms
        arm_length = 0.4
        arm_width = 0.05
        arms = []
        colors = [[0.7, 0.7, 0.7], [0.7, 0.7, 0.7], [0.7, 0.7, 0.7], [0.7, 0.7, 0.7]]
        
        for i in range(4):
            angle = i * np.pi/2
            arm = o3d.geometry.TriangleMesh.create_box(
                width=arm_length, height=arm_width, depth=arm_width)
            R = arm.get_rotation_matrix_from_xyz((0, 0, angle))
            arm.rotate(R, center=(0, 0, 0))
            arm.translate((arm_length/2, 0, 0))
            arm.paint_uniform_color(colors[i])
            arms.append(arm)
        
        # Combine meshes
        drone_mesh = body
        for arm in arms:
            drone_mesh += arm
            
        # Center the drone
        drone_mesh.translate([-0.15, -0.15, -0.05])
        
        return drone_mesh
        
    def _setup_visualization(self):
        """Setup better visualization parameters"""
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])  # Dark gray
        opt.point_size = 3.0
        opt.show_coordinate_frame = True
        
        # Setup better camera view
        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.8)
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 0, 1])
        
    def update(self, state):
        """Update visualization with new state"""
        position = state['position']
        self.trajectory.append(position)
        
        # Limit trajectory length
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)
        
        self._update_trajectory_plots()
        self._update_3d_visualization(state)
        
    def _update_trajectory_plots(self):
        """Update all trajectory plots with enhanced styling"""
        if len(self.trajectory) > 0:
            trajectory = np.array(self.trajectory)
            
            # Clear all axes
            self.ax_3d.cla()
            self.ax_xy.cla()
            self.ax_xz.cla()
            self.ax_yz.cla()
            
            # Plot 3D trajectory with single color
            self.ax_3d.plot3D(
                trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                'c-', linewidth=2
            )
            self.ax_3d.scatter(
                trajectory[-1:, 0], trajectory[-1:, 1], trajectory[-1:, 2],
                c='red', s=100
            )
            
            # Set consistent axis limits
            limit = 3.0
            self.ax_3d.set_xlim([-limit, limit])
            self.ax_3d.set_ylim([-limit, limit])
            self.ax_3d.set_zlim([0, limit])
            
            # Style enhancement
            self.ax_3d.set_title('3D Trajectory', color='white', pad=20)
            self.ax_3d.set_facecolor((0.1, 0.1, 0.1))
            self.ax_3d.grid(True, linestyle='--', alpha=0.3)
            
            # Update 2D views
            views = [
                (self.ax_xy, trajectory[:, 0], trajectory[:, 1], 'Top View (XY)'),
                (self.ax_xz, trajectory[:, 0], trajectory[:, 2], 'Front View (XZ)'),
                (self.ax_yz, trajectory[:, 1], trajectory[:, 2], 'Side View (YZ)')
            ]
            
            for ax, x, y, title in views:
                ax.plot(x, y, 'c-', linewidth=2)
                ax.scatter(x[-1:], y[-1:], c='red', s=100)
                ax.set_title(title, color='white', pad=10)
                ax.grid(True, linestyle='--', alpha=0.3)
                ax.set_facecolor((0.1, 0.1, 0.1))
                ax.set_xlim([-limit, limit])
                ax.set_ylim([-limit, limit])
            
            plt.tight_layout()
            plt.draw()
            plt.pause(0.001)
            
    def _update_3d_visualization(self, state):
        """Update Open3D visualization"""
        position = state['position']
        attitude = state['attitude']
        
        T = np.eye(4)
        T[:3, 3] = position
        
        # Apply rotation from attitude angles
        Rx = self._rotation_matrix_x(attitude[0])
        Ry = self._rotation_matrix_y(attitude[1])
        Rz = self._rotation_matrix_z(attitude[2])
        R = Rz @ Ry @ Rx
        T[:3, :3] = R
        
        # Update drone mesh transform
        self.drone_mesh.transform(T)
        
        # Update visualization
        self.vis.update_geometry(self.drone_mesh)
        self.vis.poll_events()
        self.vis.update_renderer()
        
    @staticmethod
    def _rotation_matrix_x(theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        
    @staticmethod
    def _rotation_matrix_y(theta):
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        
    @staticmethod
    def _rotation_matrix_z(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
