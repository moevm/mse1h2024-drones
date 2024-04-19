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
        self.image_subscriber = self.create_subscription(
            ROSImage, 'Mavic_2_PRO/camera/image_color', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_subscriber

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.display_image(cv_image)

    def display_image(self, cv_image):
        image = Image.fromarray(cv_image)
        image = ImageTk.PhotoImage(image)
        if not hasattr(self, 'panel'):
            self.panel = tk.Label(image=image)
            self.panel.image = image
            self.panel.pack(side="top",anchor="n")
        else:
            self.panel.configure(image=image)
            self.panel.image = image

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
                
        self.button_emergency.pack(side=tk.BOTTOM)
        self.button_end.pack(side=tk.BOTTOM)
        self.button_start.pack(side=tk.BOTTOM)

        self.node = rclpy.create_node('publisher')
        self.publisher = self.node.create_publisher(Int8, "fly",10)
        self.camera_subscriber = DroneCameraSubscriber()
        #rclpy.spin(self.camera_subscriber)
       

    def start_demo(self):
        print("Start demo")
        #self.activate_camera()
        self.fly_publisher(1)

    def end_demo(self):
        print("End demo")
        self.fly_publisher(0)

    def emergency(self):
        print("Emergency stop")
        self.activate_camera()
        self.fly_publisher(2)

    def fly_publisher(self, fly):
        msg = Int8()
        msg.data = fly
        self.publisher.publish(msg)

    def activate_camera(self):
        rclpy.spin_once(self.camera_subscriber)
        
if __name__ == "__main__":
    rclpy.init()
    root = tk.Tk()
    gui = DroneGui(root)
    root.mainloop()
    rclpy.shutdown()