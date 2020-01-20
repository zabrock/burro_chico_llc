#!/usr/bin/env python

import rospy, time
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist

class ServoConvert():
    '''
    Class that converts Twist commands into I2C control signals using
    the i2cpwm_board library.
    '''
    def __init__(self, id=1, center_value=333, servo_range=90, direction=1):
        self.value = 0.0
        self.id = id
        self.center_value = center_value

        self._center = center_value
        self._range = servo_range
        self._half_range = 0.5*servo_range
        self._dir = direction

        # Convert the range to be in the range [-1,1]
        self._sf = 1.0/self._half_range

    def convert_value(self, value_in):
        '''
        Given an input reference in range [-1,1], converts input
        to within servo range as defined at initialization.
        '''

        self.value = value_in
        self.value_out = int(self._dir*value_in*self._half_range + self._center)
        return(self.value_out)

class DkLowLevelCtrl():
    '''
    Low-level control for the Donkey Car using ROS.
    '''
    def __init__(self):
        rospy.loginfo("Setting up low-level control node...")

        rospy.init_node("dk_llc")

        # Create an actuator dictionary
        self.actuators = {}
        self.actuators['throttle'] = ServoConvert(id=1)
        self.actuators['steering'] = ServoConvert(id=2, direction=1) # Sets positive direction left

        # Create the servo arra publisher and message
        self._servo_msg = ServoArray()
        for i in range(2): # One for speed, one for steering
            self._servo_msg.servos.append(Servo()) 

        self.pub = rospy.Publisher('/servos_absolute', ServoArray, queue_size=1)
        rospy.loginfo('> LLC Publisher correctly initialized')

        # Create subscriber to the /cmd_vel topic
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo('> LLC Subscriber correctly initialized')

        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5 # Stop after this amount of seconds if no additional commands received

        rospy.loginfo('LLC initialization complete!')

    def set_actuators_from_cmdvel(self, msg):
        '''
        Receives a Twist message msg from cmd_vel; assumes maximum input in msg is 1.
        '''
        # Save the time
        self._last_time_cmd_rcv = time.time()

        # Convert vel into servo values
        self.actuators['throttle'].convert_value(msg.linear.x) # Positive is forward
        self.actuators['steering'].convert_value(msg.angular.z) # Positive is right

        # Publish the message
        self.send_servo_msg()

    def set_actuators_idle(self):
        '''
        Sets throttle and steering to zero position.
        '''
        self.actuators['throttle'].convert_value(0)
        self.actuators['steering'].convert_value(0)
        rospy.loginfo('Setting actuators idle')

        # Publish the message
        self.send_servo_msg()

    def send_servo_msg(self):
        '''
        Publish the current actuator values.
        '''
        # Save current data to servo message
        for actuator_name, servo_obj in self.actuators.items():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out

        # Publish the message
        self.pub.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):
        # Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()


if __name__ == "__main__":
    dk_llc = DkLowLevelCtrl()
    dk_llc.run()
