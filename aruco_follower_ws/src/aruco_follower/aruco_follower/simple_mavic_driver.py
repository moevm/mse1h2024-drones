"""This driver follows aruco marker."""

import math
import rclpy
from std_msgs.msg import Int8, Float32MultiArray
from .pid_controller import PIDController

K_VERTICAL_THRUST = 68.5   # with this thrust, the drone lifts.
K_VERTICAL_OFFSET = 0.3    # Vertical offset where the robot actually targets to stabilize itself.
K_VERTICAL_P = 10.0 
K_VERTICAL_I = 0.02
K_VERTICAL_D = 200

K_ROLL_P = 10.0
K_ROLL_I = 0
K_ROLL_D = 50
K_ROLL_CONST = 0.06

K_PITCH_P = 10.0   
K_PITCH_I = 0
K_PITCH_D = 50    
K_PITCH_CONST = 0.14    

K_YAW_P = -0.01
K_YAW_I = 0
K_YAW_D = -0.5

K_MARKER_P = -0.125
K_MARKER_I = 0
K_MARKER_D = -0.5

TAKEOFF_HIGHT = 0.3
TARGET_MARKER_X = -0.3

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class MavicDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Pid controllers
        self.__vertical_pid = PIDController(K_VERTICAL_P, K_VERTICAL_I, K_PITCH_D)
        self.__roll_pid = PIDController(K_ROLL_P, K_ROLL_I, K_ROLL_D)
        self.__pitch_pid = PIDController(K_PITCH_P, K_PITCH_I, K_PITCH_D)
        self.__yaw_pid = PIDController(K_YAW_P, K_YAW_I, K_YAW_D)
        self.__marker_pid = PIDController(K_MARKER_P, K_MARKER_I, K_MARKER_D)

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
        self.__fly = 2
        self.__marker_x = TARGET_MARKER_X

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('simple_mavic_driver')
        self.__node.create_subscription(Int8, 'fly', self.__fly_callback, 1)
        self.__node.create_subscription(Float32MultiArray, 'marker', self.__marker_callback, 10)
    
    def __fly_callback(self, fly_msg):
        self.__fly = fly_msg.data
    
    def __marker_callback(self, marker_msg):
        self.__marker_x = marker_msg.data[0]
    
    def compute_movement(self, target_z, target_roll, target_pitch, target_yaw, dt = 1):
        # Read sensors
        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        _, _, vertical = self.__gps.getValues()

        z_output = self.__vertical_pid.calculate(vertical, target_z, dt)
        roll_output = -self.__roll_pid.calculate(roll, target_roll, dt) + K_ROLL_CONST
        pitch_output = -self.__pitch_pid.calculate(pitch, target_pitch, dt) + K_PITCH_CONST
        yaw_output = self.__yaw_pid.calculate(yaw, target_yaw, dt)

        return z_output, roll_output, pitch_output, yaw_output
    
    def landing(self):
        return 0, 0, 0, 0
    
    def propellers_velocities(vertical_input, roll_input, pitch_input, yaw_input):
        m1 = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        m2 = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        m3 = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        m4 = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input
        return m1, m2, m3, m4
    
    def takeoff(self):
        vertical_input, roll_input, pitch_input, yaw_input = self.compute_movement(TAKEOFF_HIGHT, 0, 0, 0)
        return MavicDriver.propellers_velocities(vertical_input, roll_input, pitch_input, yaw_input)
    
    def flying(self):
        target_roll = self.__marker_pid.calculate(self.__marker_x, TARGET_MARKER_X, 1)
        vertical_input, roll_input, pitch_input, yaw_input = self.compute_movement(TAKEOFF_HIGHT, target_roll, 0, 0)
        return MavicDriver.propellers_velocities(vertical_input, roll_input, pitch_input, yaw_input)
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        m1, m2, m3, m4 = None, None, None, None

        if self.__fly == 2:
            m1, m2, m3, m4 = self.landing()
        elif self.__fly == 0:
            m1, m2, m3, m4 = self.takeoff()
        else:
            m1, m2, m3, m4 = self.flying()

        self.__propellers[0].setVelocity(m1)
        self.__propellers[1].setVelocity(-m2)
        self.__propellers[2].setVelocity(-m3)
        self.__propellers[3].setVelocity(m4)