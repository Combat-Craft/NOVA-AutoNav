**Overview:**

This Python program detects blue and red LEDs in a video feed using color and brightness thresholding. The camera captures frames, processes them to identify bright-colored LEDs, and outlines detected LEDs with rectangles.

**Code Description:**

This description outlines the main components of the code to help understand its implementation.

1. **Capture Frame from Camera**
    - The code initializes the camera feed using `cv2.VideoCapture(-1)`, which starts capturing frames from the default camera.
    - It continuously reads frames using `cap.read()` inside a loop.
2. **Convert to HSV and Grayscale**
    - The frame is converted to HSV color space using `cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)`. This helps in isolating specific colors.
    - The frame is also converted to grayscale using `cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)`, which is used to assess brightness levels.
3. **Define Color Thresholds for LEDs**
    - The program defines HSV ranges for detecting blue and red colors.
    - Red is split into two ranges (`lower_red1-upper_red1` and `lower_red2-upper_red2`) to account for hue wrapping in HSV.
    - The `cv2.inRange()` function creates masks for each color.
4. **Filter Bright Regions**
    - A brightness threshold is applied to the grayscale frame using `cv2.threshold()`, filtering out dark areas.
    - The program combines the color and brightness masks using `cv2.bitwise_and()` to ensure only bright LEDs are detected.
5. **Detect and Highlight LEDs**
    - Contours are detected using `cv2.findContours()`.
    - Small noise is filtered out by considering only contours with an area greater than 100.
    - Detected LEDs are highlighted by drawing green rectangles using `cv2.rectangle()`.
6. **Display and Control the Program**
    - The processed frame is displayed using `cv2.imshow()`.
    - The loop continues running until the user presses 'q', detected using `cv2.waitKey(1) & 0xFF == ord('q')`.
    - The program releases the camera and closes all OpenCV windows when exiting.

**Code Changes to Implement Other Functionalities:**

To modify this code for additional functionalities, such as detecting LEDs of different colors or integrating it with another system, consider the following:

- Adjust the HSV color thresholds to detect different LED colors.
- Change the brightness threshold if different lighting conditions affect detection.
- Incorporate additional OpenCV functions for real-time object tracking or logging detected LEDs.

**Code Implementation:**

To run the program:

1. Ensure you have OpenCV and NumPy installed using:

    ```
    pip install opencv-python numpy
    ```

2. Run the script:

    ```
    python3 LEDbrightness.py
    ```

3. Press 'q' to exit the program.
