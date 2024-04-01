# ArUco-markers detect application through images

### The structure of the program and instructions
- There are 3 folders: 
    + ArUco_marker_images: contains images for detecting
    + test: contains test programs of the application
    + results: initialized after running the program to contain images detected as ArUco markers
- Run the program by using the command line:
    ```python3 detect_ArUco_byImg.py```

- Some notes on the image quality after detecting Aruco markers:

    The quality of the detected images may vary due to the different sources of images captured by cameras with different specifications. In the case where a specific type of camera with specific parameters has been identified, the following indices in the ```detect_ArUco_byImg.py``` will be recalculated specifically:
        + ```cam_matrix: ``` Camera matrix, which is used to transform points in 3D space to points on a 2D screen.
        + ```dist_coeffs: ```Distortion coefficients, which are used to adjust image distortion from the lens.
