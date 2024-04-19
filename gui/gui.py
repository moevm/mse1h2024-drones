import tkinter as tk
import cv2
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Int8, Bool
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class DroneCameraSubscriber(Node):
    def __init__(self):
        super().__init__('drone_camera_subscriber')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            ROSImage, '/marked', self.image_callback, 10)
        self.image_subscriber

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Drone Camera", cv_image)
        cv2.waitKey(10)

class DroneGui:
    def __init__(self, root):
        self.root = root
        self.root.title("aruco tracker")

        self.button_start = tk.Button(root, text="Начать демонстрацию", width=20, 
                            height=5, bg='white', fg='black', command=self.start_demo)

        self.button_end = tk.Button(root, text="Закончить демонстрацию", width=20,
                            height=5, bg='white', fg='black', command=self.end_demo)

        self.button_emergency = tk.Button(root, text="Аварийная остановка", width=20, 
                                height=5, bg='white', fg='black',command=self.emergency)
                
        self.button_start.pack()
        self.button_end.pack()
        self.button_emergency.pack()

        self.node = rclpy.create_node('publisher')
        self.publisher = self.node.create_publisher(Bool, "fly",10)
        self.camera_subscriber = DroneCameraSubscriber()

    def start_demo(self):
        print("Start demo")
        self.fly_publisher(True)

    def end_demo(self):
        print("End demo")
        self.fly_publisher(False)

    def emergency(self):
        print("Emergency stop")
        self.fly_publisher(False)

    def fly_publisher(self, fly):
        msg = Bool()
        msg.data = fly
        self.publisher.publish(msg)


    
if __name__ == "__main__":
    rclpy.init()
    root = tk.Tk()
    gui = DroneGui(root)
    root.mainloop()
    rclpy.shutdown()