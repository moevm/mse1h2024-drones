"""A simple mavic driver. The only thing it can do is to takeoff or fall."""

import math
import rclpy
from std_msgs.msg import Bool


K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0
K_YAW_P = 2.0
K_VZ = 0.8
LIFT_HEIGHT = 2

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__gps.enable(self.__timestep)
        self.__gyro.enable(self.__timestep)
        self.__imu.enable(self.__timestep)

        # Enable camera
        self.__camera = self.__robot.getDevice('camera')
        self.__camera.enable(self.__timestep)

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('rear left propeller'),
            self.__robot.getDevice('rear right propeller'),
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(1)

        # State
        self.__fly = False

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('simple_mavic_driver')
        self.__node.create_subscription(Bool, 'fly', self.__fly_callback, 1)

    def __fly_callback(self, fly_msg):
        self.__fly = fly_msg.data

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Read sensors
        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        _, _, vertical = self.__gps.getValues()
        _, _, vz = self.__gps.getSpeedVector()
        roll_velocity, pitch_velocity, yaw_velocity = self.__gyro.getValues()

        vertical_ref = LIFT_HEIGHT if self.__fly else 0
        
        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity
        yaw_input = - K_YAW_P * yaw_velocity

        clamped_difference_altitude = clamp(
            vertical_ref - vertical + K_VERTICAL_OFFSET, -1, 1
        )
        vertical_input = K_VERTICAL_P * clamped_difference_altitude \
            - K_VZ * vz

        m1 = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        m2 = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        m3 = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        m4 = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        # Apply control
        self.__propellers[0].setVelocity(m1)
        self.__propellers[1].setVelocity(-m2)
        self.__propellers[2].setVelocity(-m3)
        self.__propellers[3].setVelocity(m4)
