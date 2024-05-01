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
    focal_length_x = 640
    focal_length_y = 640
    principal_point_x = 320
    principal_point_y = 240



    cam_matrix = np.array([[focal_length_x, 0, principal_point_x],[0, focal_length_y, principal_point_y],[0, 0, 1]], dtype=np.float32)

    # Distortion coefficients
    dist_coeffs = np.array([[0], [0], [0], [0], [0]], dtype=np.float32)
    
    
    #cam_matrix =  np.array([[7.34036788e+03, 0.00000000e+00, 4.19481849e+02], [0.00000000e+00, 7.29845451e+03, 2.48099654e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],dtype=np.float32)
    #dist_coeffs = np.array([[3.83599489e+01], [-2.11326505e+04],  [1.87438418e-02], [-2.65239840e-01], [-1.78689762e+01]], dtype=np.float32)
    print("cam_matrix =",cam_matrix)
    print("dist_coeefs = ", dist_coeffs)


    # Create ArUco detector object
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters_create()

    # Loop through images in the folder
    for filename in os.listdir(folder_path):
        # Read image
        image_path = os.path.join(folder_path, filename)
        frame = cv2.imread(image_path)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        # Draw markers and coordinate axes
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cam_matrix, dist_coeffs)

            if rvecs is not None and tvecs is not None:
                for i in range(len(rvecs)):
                    # Draw coordinate axes
                    frame = aruco.drawAxis(frame, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

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
