from tkinter import Tk, Label, Button, BOTTOM
from cv2 import cvtColor, COLOR_BGR2RGB
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Int8, Bool
from cv_bridge import CvBridge
from rclpy import create_node, spin, shutdown, init
from rclpy.node import Node
from threading import Thread

class DroneCameraSubscriber(Node):
    def __init__(self):
        super().__init__('drone_camera_subscriber')
        self.image_subscriber = self.create_subscription(
            ROSImage, 'marked', self.image_callback, 10) # заменить топик на /marked
        self.bridge = CvBridge()
        self.image_subscriber

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cvtColor(cv_image, COLOR_BGR2RGB)
        self.display_image(cv_image)

    def display_image(self, cv_image):
        image = Image.fromarray(cv_image)
        image = ImageTk.PhotoImage(image)
        if not hasattr(self, 'panel'):
            self.panel = Label(image=image)
            self.panel.image = image
            self.panel.pack(side="top",anchor="n")
        else:
            self.panel.configure(image=image)
            self.panel.image = image

class DroneGui:
    def __init__(self, root):
        self.root = root
        self.root.title("aruco tracker")

        self.button_start = Button(root, text="Лететь", width=20, 
                            height=5, bg='white', fg='black', command=self.start_demo)

        self.button_end = Button(root, text="Снижаться", width=20,
                            height=5, bg='white', fg='black', command=self.end_demo)

        self.button_emergency = Button(root, text="Остановиться", width=20, 
                                height=5, bg='white', fg='black',command=self.emergency)
                
        self.button_emergency.pack(side=BOTTOM)
        self.button_end.pack(side=BOTTOM)
        self.button_start.pack(side=BOTTOM)

        self.node = create_node('publisher')
        self.publisher = self.node.create_publisher(Int8, "fly", 10)
        self.camera_subscriber = DroneCameraSubscriber()
       

    def start_demo(self):
        print("Start demo")
        self.fly_publisher(1)

    def end_demo(self):
        print("End demo")
        self.fly_publisher(0)

    def emergency(self):
        print("Emergency stop")
        self.fly_publisher(2)

    def fly_publisher(self, fly):
        msg = Int8()
        msg.data = fly
        self.publisher.publish(msg)

        
if __name__ == "__main__":
    init()
    root = Tk()
    gui = DroneGui(root)
    spin_thread=Thread(target=spin, args=(gui.camera_subscriber,))
    spin_thread.start()
    root.mainloop()
    shutdown()
