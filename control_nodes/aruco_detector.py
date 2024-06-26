import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2.aruco as aruco
import numpy as np

class Marker:
    def __init__(self, marker_id, position):
        self.marker_id = marker_id
        self.position = position

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.subscription = self.create_subscription(
            Image,
            'Mavic_2_PRO/camera/image_color',  
            self.image_callback,
            10) #take image from topic /Image

        self.publisher = self.create_publisher(Image, '/marked', 10)  # Send processed image to /marked
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(Float32MultiArray, '/marker', 10) # Send coordinates to /marker

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        marked_image = self.detect_aruco_markers(cv_image)
        marked_msg = self.bridge.cv2_to_imgmsg(marked_image, encoding="bgr8")
        self.publisher.publish(marked_msg)

    def detect_aruco_markers(self, image):

        # Camera dimensions and calibration parameters
        focal_length_x = 880.76258376
        focal_length_y = 879.79177239
        principal_point_x = 331.61928164
        principal_point_y = 185.48140499
        cam_matrix = np.array([[focal_length_x, 0, principal_point_x],
                               [0, focal_length_y, principal_point_y],
                               [0, 0, 1]], dtype=np.float32)

        # Distortion coefficients (default values)
        dist_coeffs = np.array([[-2.12846470e-01], [3.91151145e+00], [-1.82790992e-04], [1.48727617e-03], [-2.32401748e+01]], dtype=np.float32)
       

        # Create a custom dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        parameters = aruco.DetectorParameters()
   

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw markers and coordinate axes
        if ids is not None:
            marked_image = image.copy()  
            image = aruco.drawDetectedMarkers(image, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cam_matrix, dist_coeffs)

            if rvecs is not None and tvecs is not None:
                for i in range(len(rvecs)):
                    # Get marker position
                    marker_position = tvecs[i][0]

                    # Create and publish marker message
                    msg = Float32MultiArray()
                    msg.data = marker_position
                    self.marker_publisher.publish(msg)

                    # Draw marker ID and coordinate axes
                    cv2.putText(image, f"ID: {ids[i]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.drawFrameAxes(marked_image, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
                return marked_image
        return image

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
