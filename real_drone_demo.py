from djitellopy import Tello
import cv2
import numpy as np
import time
from enhanced_slam import EnhancedSLAM

class RealDroneDemo:
    def __init__(self):
        # Initialize drone
        self.drone = Tello()
        self.drone.connect()
        print(f"Battery Level: {self.drone.get_battery()}%")
        
        # Initialize SLAM
        self.slam = EnhancedSLAM()
        
        # Start video stream
        self.drone.streamon()
        time.sleep(2)  # Wait for stream to start
        
    def run_demo(self):
        print("Starting Demo...")
        print("Commands:")
        print("- Press 'T' to takeoff")
        print("- Press 'L' to land")
        print("- Press 'Q' to quit")
        print("- Use arrow keys to move")
        
        try:
            while True:
                # Get drone data
                frame = self.drone.get_frame_read().frame
                
                # Basic IMU data
                imu_data = {
                    'acceleration': np.array([0., 0., 0.]),
                    'gyro': np.array([0., 0., 0.]),
                    'dt': 0.1
                }
                
                # Process with SLAM
                pose = self.slam.process_frame(frame, imu_data)
                
                # Show drone camera feed
                cv2.imshow("Drone Camera", frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xff
                if key == ord('q'):
                    break
                elif key == ord('t'):
                    self.drone.takeoff()
                elif key == ord('l'):
                    self.drone.land()
                elif key == 82:  # Up arrow
                    self.drone.move_forward(30)
                elif key == 84:  # Down arrow
                    self.drone.move_back(30)
                elif key == 81:  # Left arrow
                    self.drone.move_left(30)
                elif key == 83:  # Right arrow
                    self.drone.move_right(30)
                
                time.sleep(0.1)
                
        finally:
            # Cleanup
            self.drone.streamoff()
            self.drone.land()
            cv2.destroyAllWindows()
            
if __name__ == "__main__":
    demo = RealDroneDemo()
    demo.run_demo()
