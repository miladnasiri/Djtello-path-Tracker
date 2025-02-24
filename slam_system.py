import numpy as np
import cv2
from filterpy.kalman import ExtendedKalmanFilter
import open3d as o3d

class SLAMSystem:
    def __init__(self):
        self.feature_detector = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8)
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=6)
        self.setup_ekf()
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.map_descriptors = None
        
        self.camera_matrix = np.array([
            [921.170702, 0.000000, 459.904354],
            [0.000000, 919.018377, 351.238301],
            [0.000000, 0.000000, 1.000000]
        ])
        self.dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
        
        self.point_cloud = o3d.geometry.PointCloud()

    def setup_ekf(self):
        self.ekf.F = np.eye(12)
        self.ekf.H = np.zeros((6, 12))
        self.ekf.H[:3, :3] = np.eye(3)
        self.ekf.H[3:, 6:9] = np.eye(3)
        self.ekf.R = np.eye(6) * 0.1
        self.ekf.Q = np.eye(12) * 0.01
        self.ekf.P = np.eye(12) * 0.1
        self.ekf.x = np.zeros(12)

    def process_frame(self, frame, imu_data):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.detect_features(gray)
        
        if len(self.map_points) > 0:
            matches = self.match_features(descriptors)
            pose = self.estimate_pose(keypoints, matches)
        else:
            self.initialize_map(keypoints, descriptors)
            pose = np.eye(4)
            
        self.update_ekf(imu_data, pose)
        self.update_map(keypoints, pose)
        return pose

    def detect_features(self, frame):
        keypoints = self.feature_detector.detect(frame, None)
        return self.feature_detector.compute(frame, keypoints)

    def match_features(self, descriptors):
        if self.map_descriptors is None: return []
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(descriptors, self.map_descriptors)
        return [m for m in matches if m.distance < 50]

    def estimate_pose(self, keypoints, matches):
        if len(matches) < 4: return np.eye(4)
        
        try:
            obj_points = np.array([self.map_points[m.trainIdx] for m in matches], 
                                  dtype=np.float32).reshape(-1, 3)
            img_points = np.array([keypoints[m.queryIdx].pt for m in matches], 
                                  dtype=np.float32).reshape(-1, 2)
            
            if obj_points.shape[0] < 4: return np.eye(4)
            
            success, rvec, tvec, _ = cv2.solvePnPRansac(
                obj_points, img_points, 
                self.camera_matrix, self.dist_coeffs,
                confidence=0.99, reprojectionError=3.0
            )
            
            if not success or tvec[2] < 0: return np.eye(4)
            
            R, _ = cv2.Rodrigues(rvec)
            return self.create_transform_matrix(R, tvec.ravel())
        except:
            return np.eye(4)

    def initialize_map(self, keypoints, descriptors):
        if len(keypoints) == 0: return
        self.map_points = np.array([(kp.pt[0], kp.pt[1], 1.0) for kp in keypoints], 
                                  dtype=np.float32)
        self.map_descriptors = descriptors

    def update_ekf(self, imu_data, pose):
        dt = imu_data.get('dt', 0.1)
        self.ekf.F[:3, 3:6] = np.eye(3) * dt
        self.ekf.predict()
        
        def HJacobian(x): return self.ekf.H
        def Hx(x): return self.ekf.H @ x
        
        z = np.concatenate([pose[:3, 3], self.rotation_to_euler(pose[:3, :3])])
        self.ekf.update(z, HJacobian, Hx)

    def update_map(self, keypoints, pose):
        if len(keypoints) == 0: return
        
        try:
            # Fixed parenthesis closure
            new_points = np.hstack((
                np.array([(kp.pt[0], kp.pt[1], 1.0) for kp in keypoints], dtype=np.float32),
                np.ones((len(keypoints), 1))
            ))
            
            transformed = (pose @ new_points.T).T[:, :3]
            valid = transformed[:, 2] > 0.1
            world_points = transformed[valid]
            
            if world_points.size > 0:
                self.map_points = np.vstack((self.map_points, world_points))
                self.point_cloud.points = o3d.utility.Vector3dVector(self.map_points)
        except Exception as e:
            print(f"Map update error: {e}")

    def get_map(self):
        return self.point_cloud

    @staticmethod
    def create_transform_matrix(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    @staticmethod
    def rotation_to_euler(R):
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6
        x = np.arctan2(R[2,1], R[2,2]) if not singular else np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0]) if not singular else 0
        return np.array([x, y, z])
