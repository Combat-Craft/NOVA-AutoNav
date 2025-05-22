## **2. Text Detection with EasyOCR**

Text recognition allows extraction of meaningful information from images or video feeds. This section details using EasyOCR for text detection and translation.

[main.py](https://prod-files-secure.s3.us-west-2.amazonaws.com/2e02a0e1-d5ce-4f6c-bcbc-965ac3615cc5/61f246fe-047f-43e3-bb45-5e0da922288e/main.py)

### **Compatibility Issue**

EasyOCR (v2.5.1) is incompatible with CUDA Toolkit 12.2, causing GPU acceleration issues.

- **Options:**
    1. Build PyTorch with CUDA support for EasyOCR.
    2. Use alternative frameworks like PaddleOCR or Tesseract OCR.

### **Steps for EasyOCR Detection**

1. **Install Dependencies:**

    ```bash
    pip3 install easyocr googletrans==4.0.0-rc1 opencv-python-headless rospy

    ```

2. **Run Text Detection Script:**
Save the provided `main.py` script and execute:

    ```bash
    python3 main.py

    ```

3. **Key Features:**
    - Detects text from the video feed using EasyOCR.
    - Translates detected text into multiple languages using Google Translate.
    - Publishes results to the ROS topic `detected_text`.
4. **Alternative Frameworks:**
If GPU support is critical, consider PaddleOCR or Tesseract OCR.
