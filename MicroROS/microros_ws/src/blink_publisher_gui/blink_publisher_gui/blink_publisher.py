import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PySide6.QtWidgets import QMainWindow 
from PySide6 import QtWidgets
from blink_publisher_gui.gui.Window_ui import *
import sys

class BlinkPublisher(Node):
    def __init__(self):
        super().__init__('blink_publisher')
        self.publisher_ = self.create_publisher(Int32, 'blink_topic', 10)

    def publish_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        QMainWindow.__init__(self, *args, **kwargs)
        self.setupUi(self)
        rclpy.init()
        self.blink_publisher = BlinkPublisher()
        self.value = 0
        self.pushButton.clicked.connect(self.click_button)
        
    def click_button(self):
        if self.value == 0: 
            self.value = 1
            self.pushButton.setText("Apagar LED")
        else: 
            self.value = 0
            self.pushButton.setText("Encender LED")
            
        self.blink_publisher.publish_message(self.value)
        
        
    def closeEvent(self, event):
        self.blink_publisher.destroy_node()
        rclpy.shutdown()
        return super().closeEvent(event)

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

    
if __name__ == "__main__":
    main()    