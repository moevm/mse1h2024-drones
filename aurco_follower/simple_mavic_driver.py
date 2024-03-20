"""A simple mavic driver. The only thing it can do is to takeoff or fall."""

import math
import rclpy
from std_msgs.msg import Bool


VERTICAL_THRUST_NEUTRAL = 68.5    # with this thrust, the drone lifts.
VERTICAL_THRUST_POSITIVE = 70
LIFT_HEIGHT = 2


class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

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
        _, _, vertical = self.__gps.getValues()

        thrust = VERTICAL_THRUST_NEUTRAL
        if self.__fly:
            if vertical < LIFT_HEIGHT:
                thrust = VERTICAL_THRUST_POSITIVE
        else:
            thrust = 0

        # Apply control
        self.__propellers[0].setVelocity(-thrust)
        self.__propellers[1].setVelocity(thrust)
        self.__propellers[2].setVelocity(thrust)
        self.__propellers[3].setVelocity(-thrust)