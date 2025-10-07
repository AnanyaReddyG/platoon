import numpy as np
import cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Header, Int32MultiArray
from scipy.spatial.transform import Rotation as R

class MultiArucoTracker(Node):
    def __init__(self):
        super().__init__('multi_aruco_tracker')
        
        # Create publishers for detected aruco poses and their IDs
        self.pose_pub = self.create_publisher(PoseArray, '/burger3/aruco_poses', 10)
        self.id_pub = self.create_publisher(Int32MultiArray, '/burger3/aruco_ids', 10)
        
        # Initialize the Intel RealSense camera pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        
        # Fetch camera intrinsics for pose estimation
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intrinsics = color_profile.get_intrinsics()
        self.camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array(intrinsics.coeffs)
        
        # Set up ArUco detection logic
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        self.marker_size = 0.1 # Marker size in meters
        
        # Schedule the processing of camera frames at 10 Hz
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        try:
            # Grab the latest frame from the camera
            frames = self.pipeline.wait_for_frames(1000)
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return
            
            color_image = np.asanyarray(color_frame.get_data())
            
            # Detect markers in the current camera image
            corners, ids, _ = self.detector.detectMarkers(color_image)
            if ids is not None:
                # Estimate pose (position/orientation) for each detected marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                # Create PoseArray and ID message to publish
                pose_array = PoseArray()
                id_array = Int32MultiArray()
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "camera_link"
                pose_array.header = header
                id_array.data = ids.flatten().tolist()
                
                for i in range(len(ids)):
                    pose = Pose()
                    # Set translation (position) component from tvecs
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][0][1]
                    pose.position.z = tvecs[i][0][2]
                    # Set orientation as quaternion from rotation vector
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    quat = R.from_matrix(rotation_matrix).as_quat()
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]
                    pose_array.poses.append(pose)
                
                # Publish array of marker poses and their IDs
                self.pose_pub.publish(pose_array)
                self.id_pub.publish(id_array)
                
                # Optional visualization: draw detected markers & axes for debugging
                debug_image = cv2.aruco.drawDetectedMarkers(color_image.copy(), corners, ids)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        debug_image, self.camera_matrix, self.dist_coeffs,
                        rvecs[i], tvecs[i], 0.1
                    )
                cv2.imshow("Multi-Marker Tracking", debug_image)
                cv2.waitKey(1)
        except Exception as e:
            # Log error if any failures in frame processing
            self.get_logger().error(f"Processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release camera and node resources on exit
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
