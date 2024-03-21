import unittest
import cv2
import numpy as np
from poseEstimate import main

class TestMainProgram(unittest.TestCase):
    def setUp(self):
        # Set up camera parameters
        self.focal_length_x = 640
        self.focal_length_y = 640
        self.principal_point_x = 320
        self.principal_point_y = 240
        self.cam_matrix = np.array([[self.focal_length_x, 0, self.principal_point_x],
                                    [0, self.focal_length_y, self.principal_point_y],
                                    [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def test_detectMarkers(self):
        # Test detectMarkers function
        mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_corners = np.array([[[[100, 100]], [[200, 100]], [[200, 200]], [[100, 200]]],
                                 [[[300, 100]], [[400, 100]], [[400, 200]], [[300, 200]]]], dtype=np.float32)
        mock_ids = np.array([[0], [1]])
        
        # Call detectMarkers function
        corners, ids, _ = cv2.aruco.detectMarkers(mock_frame, self.aruco_dict, parameters=self.parameters)
        # Check if detected corners and IDs match the expected values
        self.assertTrue(np.array_equal(corners, mock_corners))
        self.assertTrue(np.array_equal(ids, mock_ids))

    def test_estimatePoseSingleMarkers(self):
        # Test estimatePoseSingleMarkers function
        # Create mock data for rvecs and tvecs
        mock_rvecs = [np.array([[0.1, 0.2, 0.3]]), np.array([[0.4, 0.5, 0.6]])]
        mock_tvecs = [np.array([[1, 2, 3]]), np.array([[4, 5, 6]])]
        corners = np.array([[[[100, 100]], [[200, 100]], [[200, 200]], [[100, 200]]],
                            [[[300, 100]], [[400, 100]], [[400, 200]], [[300, 200]]]], dtype=np.float32)
        # Call estimatePoseSingleMarkers function
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.cam_matrix, self.dist_coeffs)
        # Check if estimated rvecs and tvecs match the expected values
        for i in range(len(rvecs)):
            self.assertTrue(np.allclose(rvecs[i], mock_rvecs[i]))
            self.assertTrue(np.allclose(tvecs[i], mock_tvecs[i]))

if __name__ == '__main__':
    unittest.main()

