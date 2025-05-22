import cv2
import rospy
from std_msgs.msg import String
import easyocr
from googletrans import Translator

def detect_and_translate(frame, reader, translator, languages):
    """
    Detect text in the frame and translate it to the target languages.

    Args:
        frame (numpy.ndarray): The video frame to process.
        reader (easyocr.Reader): The EasyOCR reader for text detection.
        translator (googletrans.Translator): Google Translate API for translation.
        languages (list): List of target languages for translation (e.g., ['en', 'zh-cn', 'ru']).

    Returns:
        dict: Detected text and translations.
    """
    results = reader.readtext(frame)
    detected_texts = [res[1] for res in results]  # Extract recognized text
    translations = {}

    for text in detected_texts:
        translations[text] = {
            lang: translator.translate(text, dest=lang).text for lang in languages
        }
    return translations

def publish_translations(pub, translations):
    """
    Publish detected and translated text to a ROS topic.

    Args:
        pub (rospy.Publisher): ROS publisher object.
        translations (dict): Detected text and their translations.
    """
    for original, translated in translations.items():
        msg = f"Detected: {original}\n" + "\n".join(
            [f"{lang.upper()}: {text}" for lang, text in translated.items()]
        )
        rospy.loginfo(msg)
        pub.publish(msg)

def main():
    # ROS Node and Publisher
    rospy.init_node('text_detection_node', anonymous=True)
    pub = rospy.Publisher('detected_text', String, queue_size=10)

    # Initialize EasyOCR reader and Google Translate
    reader = easyocr.Reader(['en', 'zh', 'ru'])  # Add desired languages
    translator = Translator()

    # OpenCV Video Capture
    cap = cv2.VideoCapture(0)  # Change to video file path if needed

    rospy.loginfo("Text detection node started. Press 'q' to exit.")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame. Exiting...")
            break

        # Text Detection and Translation
        translations = detect_and_translate(frame, reader, translator, ['en', 'zh-cn', 'ru'])

        # Publish to ROS Topic
        publish_translations(pub, translations)

        # Display Frame
        for res in translations:
            cv2.putText(
                frame, res, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
            )
        cv2.imshow("Text Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
