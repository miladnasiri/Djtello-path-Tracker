import numpy as np
import cv2
from filterpy.kalman import ExtendedKalmanFilter
import open3d as o3d
import matplotlib.pyplot as plt

class EnhancedSLAM:
    def __init__(self):
        # Feature Detection
        self.feature_detector = cv2.ORB_create(
            nfeatures=1000,
            scaleFactor=1.2,
            nlevels=8
        )
        
        # Extended Kalman Filter Setup
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=6)
        self.setup_ekf()
        
        # 3D Mapping
        self.map_points = np.empty((0, 3))
        self.map_descriptors = None
        self.point_cloud = o3d.geometry.PointCloud()
        
        # Initialize previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        
        # Visualization setup
        self.setup_visualization()
        
    def setup_ekf(self):
        """Setup Extended Kalman Filter parameters"""
        self.ekf.F = np.eye(12)
        self.ekf.H = np.zeros((6, 12))
        self.ekf.H[:3, :3] = np.eye(3)
        self.ekf.H[3:, 6:9] = np.eye(3)
        self.ekf.R = np.eye(6) * 0.1
        self.ekf.Q = np.eye(12) * 0.01
        self.ekf.P = np.eye(12) * 0.1
        
    def setup_visualization(self):
        """Setup visualization windows"""
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        
        self.ax_features = self.fig.add_subplot(221)
        self.ax_features.set_title('Feature Detection')
        
        self.ax_map = self.fig.add_subplot(222, projection='3d')
        self.ax_map.set_title('3D Point Cloud Map')
        
        self.ax_matches = self.fig.add_subplot(223)
        self.ax_matches.set_title('Feature Matching')
        
        self.ax_state = self.fig.add_subplot(224)
        self.ax_state.set_title('EKF State Estimation')
        
        plt.tight_layout()
        
    def process_frame(self, frame, imu_data):
        """Process a new frame"""
        if frame is None:
            return np.eye(4)
            
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect features
        keypoints, descriptors = self.detect_features(gray)
        
        # Store current frame for next iteration
        self.prev_frame = gray
        self.prev_keypoints = keypoints
        
        # Initialize pose
        pose = np.eye(4)
        
        # Update visualization
        self.update_visualization(frame, keypoints, None)
        
        return pose
        
    def detect_features(self, frame):
        """Detect ORB features"""
        keypoints = self.feature_detector.detect(frame, None)
        keypoints, descriptors = self.feature_detector.compute(frame, keypoints)
        return keypoints, descriptors if descriptors is not None else np.array([])
        
    def update_visualization(self, frame, keypoints, matches):
        """Update all visualization windows"""
        # Draw features
        frame_with_kp = cv2.drawKeypoints(frame, keypoints, None, 
                                        color=(0, 255, 0), 
                                        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        self.ax_features.cla()
        self.ax_features.imshow(cv2.cvtColor(frame_with_kp, cv2.COLOR_BGR2RGB))
        
        # Draw 3D map
        self.ax_map.cla()
        if len(self.map_points) > 0:
            self.ax_map.scatter(self.map_points[:, 0], 
                              self.map_points[:, 1], 
                              self.map_points[:, 2], 
                              c='b', marker='.')
            
        # Update plot
        plt.draw()
        plt.pause(0.01)
