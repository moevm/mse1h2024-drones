import cv2
import cv2.aruco as aruco
import numpy as np

def main():
    # Initialize camera
    cap = cv2.VideoCapture(0) #value 0 for default camera

    # Camera dimensions and calibration parameters (default values)
    focal_length_x = 640
    focal_length_y = 640
    principal_point_x = 320
    principal_point_y = 240

    # Camera matrix to transform 3D to 2D
    cam_matrix = np.array([[focal_length_x, 0, principal_point_x],
                           [0, focal_length_y, principal_point_y],
                           [0, 0, 1]], dtype=np.float32)

    # Distortion coefficients
    dist_coeffs = np.zeros((5, 1))

    # Create ArUco detector object
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    while True:
        # Read frame from camera
        ret, frame = cap.read()

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

        # Display frame
        cv2.imshow('Frame', frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

