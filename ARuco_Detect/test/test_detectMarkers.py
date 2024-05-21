import os
import cv2
import unittest
import numpy as np
import cv2.aruco as aruco

class TestDetectMarkers(unittest.TestCase):
    def test_detectMarkers(self):
        # Using image number 4 to test this case
        input_image_path = "../ArUco_marker_images/4.jpg"
        output_image_path = "../results/4.jpg"

        # Check if the result image file exists
        self.assertTrue(os.path.exists(output_image_path), "Result image does not exist.")

        # Read the original image and the result image
        input_image = cv2.imread(input_image_path)
        output_image = cv2.imread(output_image_path)

        # Convert images to grayscale
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        # Create an ArUco dictionary object
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # Create a DetectorParameters object
        parameters = aruco.DetectorParameters_create()

        # Detect ArUco markers in the original image
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if any ArUco markers are detected
        self.assertIsNotNone(ids, "No ArUco markers detected.")

        # Check if the number of detected ArUco markers matches the number in the result image
        self.assertEqual(len(ids), 1, "Number of detected markers does not match.")

if __name__ == '__main__':
    unittest.main()
