import cv2
import cv2.aruco as aruco
import numpy as np
import os
from djitellopy import Tello

def main():
    # Initialize Tello drone
    tello = Tello()

    # Connect to the drone
    tello.connect()


    # Folder to save results
    results_folder = "results"
    if not os.path.exists(results_folder):
        os.makedirs(results_folder)

    # Camera dimensions and calibration parameters (default values)
    focal_length_x = 640
    focal_length_y = 640
    principal_point_x = 320
    principal_point_y = 240

    cam_matrix = np.array([[focal_length_x, 0, principal_point_x],
                           [0, focal_length_y, principal_point_y],
                           [0, 0, 1]], dtype=np.float32)

    # Distortion coefficients
    dist_coeffs = np.array([[0], [0], [0], [0], [0]], dtype=np.float32)

    # Create ArUco detector object
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # Loop through frames from Tello drone
    while (tello.is_streaming):
        # Get frame from Tello drone
        frame = tello.get_frame_read().frame

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
        result_image_path = os.path.join(results_folder, "frame_{}.jpg".format(tello.get_frame_read().last_frame_time))
        cv2.imwrite(result_image_path, frame)


    print("Results saved in 'results' folder.")

    # Disconnect from the drone
    tello.end()


if __name__ == "__main__":
    main()
