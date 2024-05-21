import numpy as np
import os
from cv2.aruco import DICT_5X5_1000, getPredefinedDictionary, DetectorParameters, \
    ArucoDetector, drawDetectedMarkers, estimatePoseSingleMarkers
from cv2 import imread, imwrite, drawFrameAxes

IMAGES_PATH = 'images'
RESULTS_PATH = 'results'

CAMERA_MATRIX = np.array(
    [
        [880.76258467, 0., 331.61928196],
        [0., 879.79177331, 185.48140499],
        [0., 0., 1.]
    ], dtype = np.float32
)

DISTORTION_COEFFICIENT = np.array(
    [
        [-2.12846471e-01],
        [3.91151148e+00],
        [-1.82790990e-04],
        [1.48727617e-03],
        [-2.32401751e+01]
    ], dtype = np.float32
)

if __name__ == '__main__':

    if not os.path.exists(RESULTS_PATH):
        os.makedirs(RESULTS_PATH)

    aruco_dict = getPredefinedDictionary(DICT_5X5_1000)
    parameters = DetectorParameters()
    detector = ArucoDetector(aruco_dict, parameters)

    for filename in os.listdir(IMAGES_PATH):
        
        image = imread(
            os.path.join(IMAGES_PATH, filename)
        )

        corners, ids, _ = detector.detectMarkers(image)

        if ids is not None:
            image = drawDetectedMarkers(image, corners, ids)

            rvecs, tvecs, _ = estimatePoseSingleMarkers(
                corners, 0.05, CAMERA_MATRIX, DISTORTION_COEFFICIENT
            )

            if rvecs is not None and tvecs is not None:
                for i in range(len(rvecs)):
                    image = drawFrameAxes(
                        image, CAMERA_MATRIX, DISTORTION_COEFFICIENT,
                        rvecs[i], tvecs[i], 0.1
                    )

        result_image_path = os.path.join(RESULTS_PATH, filename)
        imwrite(result_image_path, image)