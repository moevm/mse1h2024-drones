import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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
            '/camera',  # Topic nhận hình ảnh từ camera
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/marked', 10)  # Topic gửi hình ảnh đã được xử lý
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(Marker, '/marker', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        marked_image = self.detect_aruco_markers(cv_image)
        marked_msg = self.bridge.cv2_to_imgmsg(marked_image, encoding="bgr8")
        self.publisher.publish(marked_msg)

    def detect_aruco_markers(self, image):
        # Camera dimensions and calibration parameters (default values)
        # TODO: Thay thế giá trị mặc định bằng thông số hiệu chỉnh camera thực tế
        focal_length_x = 640
        focal_length_y = 640
        principal_point_x = 320
        principal_point_y = 240
        cam_matrix = np.array([[focal_length_x, 0, principal_point_x],
                               [0, focal_length_y, principal_point_y],
                               [0, 0, 1]], dtype=np.float32)

        # Distortion coefficients (default values)
        dist_coeffs = np.array([[0], [0], [0], [0], [0]], dtype=np.float32)

        # Create ArUco detector object
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)

        # Draw markers and coordinate axes
        if ids is not None:
            marked_image = image.copy()  # Tạo bản sao để tránh làm thay đổi ảnh gốc
            image = aruco.drawDetectedMarkers(image, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cam_matrix, dist_coeffs)

            if rvecs is not None and tvecs is not None:
                for i in range(len(rvecs)):
                    # Get marker position
                    marker_position = tvecs[i][0]

                    # Create and publish marker message
                    marker_msg = Marker(marker_id=ids[i], position=marker_position)
                    self.marker_publisher.publish(marker_msg)

                    # Draw marker ID and coordinate axes
                    cv2.putText(image, f"ID: {ids[i]}", (int(corners[i][0][0][0]), int(corners[i][0][0][1])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.drawAxis(marked_image, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

        return marked_image

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()