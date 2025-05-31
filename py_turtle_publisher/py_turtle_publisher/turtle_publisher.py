import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

import cv2
import os
import requests
import sys


class TurtlePublisher(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        model = 'gesture_recognizer.task'
        if not os.path.exists(model):
            url = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
            with open(model, 'wb') as out_file:
                content = requests.get(url, stream=True).content
                out_file.write(content)

        self.base_options = python.BaseOptions(model_asset_path=model)
        self.options = vision.GestureRecognizerOptions(base_options=self.base_options)
        self.recognizer = vision.GestureRecognizer.create_from_options(self.options)
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.publisher = self.create_publisher(String, 'command', 10)

        self.camera = cv2.VideoCapture(0)

        self.init_gesture_recognizer()

    def init_gesture_recognizer(self):
        if self.camera.isOpened():
            msg = String()
            while cv2.waitKey(1) != 27:
                ret, frame = self.camera.read()
                frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if ret:
                    image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frameRGB)
                    recognition_result = self.recognizer.recognize(image)
                    if len(recognition_result.gestures) > 0:
                        gesture = recognition_result.gestures[0][0]
                        info = [gesture.category_name, gesture.score]
                        msg.data = f'{info}'
                        self.publisher.publish(msg)
                        self.get_logger().info(f'Publishing: {msg.data}')
                        
                        cv2.putText(frame, info[0], (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                    
                    if len(recognition_result.hand_landmarks) > 0:
                        hand_landmarks = recognition_result.hand_landmarks
                        #msg.data = f'Hand Landmarks: {hand_landmarks}'
                        #self.publisher_.publish(msg)
                        #self.get_logger().info(f'Publishing: {msg.data}')
                        
                        for _hand_landmarks in hand_landmarks:
                            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                            hand_landmarks_proto.landmark.extend([
                                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in _hand_landmarks])

                            self.mp_drawing.draw_landmarks(
                                frame,
                                hand_landmarks_proto,
                                self.mp_hands.HAND_CONNECTIONS,
                                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                self.mp_drawing_styles.get_default_hand_connections_style())
                        
                    cv2.imshow("Video", frame)
            
            self.camera.release()
            cv2.destroyAllWindows()
            sys.exit(0)
            


def main(args=None):
    rclpy.init(args=args)

    turtle_publisher = TurtlePublisher()

    rclpy.spin(turtle_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
