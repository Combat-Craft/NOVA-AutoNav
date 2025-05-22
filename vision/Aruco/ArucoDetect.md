ArUco markers are used for robust detection and tracking in computer vision applications, such as robotics. This section explains how to detect ArUco markers and optionally perform camera calibration for pose estimation.

### **Steps for Marker Detection**

1. **Prerequisites:**
    - Python 3.x
    - OpenCV with `aruco` module

    ```bash
    sudo apt-get install python3-opencv

    ```

2. **Run Detection Script:**
Save the provided `ArucoDetect.py` script and run:

    ```bash
    python3 ArucoDetect.py

    ```

3. **Key Script Details:**
    - Uses `cv2.aruco.DICT_4X4_100` dictionary.
    - Highlights detected markers with bounding boxes and IDs on a live video feed.
4. **Optional: Camera Calibration**
For applications needing pose estimation:
    - Use `camera_calibration.py` to generate calibration data.
    - Outputs: Camera matrix (`K`) and distortion coefficients (`D`).
