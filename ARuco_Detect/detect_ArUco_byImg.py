import cv2
import cv2.aruco as aruco
import numpy as np
import os

def main():
    # Folder containing ArUco marker images
    folder_path = "ArUco_marker_images"
    # Folder to save results
    results_folder = "results"
    if not os.path.exists(results_folder):
        os.makedirs(results_folder)

    # Camera dimensions and calibration parameters (default values)
    focal_length_x = 880.76258376
    focal_length_y = 879.79177239
    principal_point_x = 331.61928164
    principal_point_y = 185.48140499
    
    cam_matrix = np.array([[focal_length_x, 0, principal_point_x],[0, focal_length_y, principal_point_y],[0, 0, 1]], dtype=np.float32)

    # Distortion coefficients
    dist_coeffs = np.array([[-2.12846470e-01], [3.91151145e+00], [-1.82790992e-04], [1.48727617e-03], [-2.32401748e+01]], dtype=np.float32)






    # Create ArUco detector object
    marker_size = 5
    dictionary_values = 1000 # Number of markers in the dictionary

    # Create a custom dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters()

    # Loop through images in the folder
    for filename in os.listdir(folder_path):
        # Read image
        image_path = os.path.join(folder_path, filename)
        frame = cv2.imread(image_path)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw markers and coordinate axes
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cam_matrix, dist_coeffs)

            if rvecs is not None and tvecs is not None:
                for i in range(len(rvecs)):
                    # Draw coordinate axes
                    frame = cv2.drawFrameAxes(frame, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
                     
                    # Get marker position
                    marker_position = tvecs[i][0]

                    # Draw names of axes
                    cv2.putText(frame, "X", (int(corners[i][0][0][0]) + 10, int(corners[i][0][0][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (0, 0, 255), 2)
                    cv2.putText(frame, "Y", (int(corners[i][0][0][0]) + 10, int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (0, 255, 0), 2)
                    cv2.putText(frame, "Z", (int(corners[i][0][0][0]) - 20, int(corners[i][0][0][1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 0, 0), 2)

        # Save result image
        result_image_path = os.path.join(results_folder, filename)
        cv2.imwrite(result_image_path, frame)

    print("Results saved in 'results' folder.")

if __name__ == "__main__":
    main()
