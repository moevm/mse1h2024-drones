import os
import cv2
import numpy as np
import unittest
import sys
sys.path.append("..")
from detect_ArUco_byImg import main


class TestSaveResults(unittest.TestCase):
    def test_saveResults(self):
        input_folder = "../ArUco_marker_images"
        output_folder = "../results"

        image_files = os.listdir(input_folder)

        for image_file in image_files:
            input_image_path = os.path.join(input_folder, image_file)
            output_image_path = os.path.join(output_folder, image_file)


            self.assertTrue(os.path.exists(output_image_path), f"File {image_file} was not saved.")

            input_image = cv2.imread(input_image_path)
            output_image = cv2.imread(output_image_path)
            self.assertFalse(np.array_equal(input_image, output_image), f"Image {image_file} was not processed.")

if __name__ == '__main__':
    unittest.main()
