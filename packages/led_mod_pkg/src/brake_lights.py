#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String, Float32

class Brake_Lights:
    def __init__(self):
        rospy.Subscriber("car_cmd_switch_node/cmd", Twist2DStamped, self.callback)
        self.obj = rospy.ServiceProxy("led_emitter_node/set_pattern", ChangePattern)
        self.current_reverse = 0
        self.past_reverse = 0
        temp_string = String()
        temp_string.data = "RED"
        #self.obj("pattern_name: {data: RED}")
        self.obj(temp_string)
        rospy.loginfo("Started")

    def callback(self, data):
        #self.obj("pattern_name: {data: RED}")
        #return
        if data.v <= 0:
            self.current_reverse = 1
        else:
            self.current_reverse = 0
        if self.current_reverse == self.past_reverse:
            return

        pattern_str = String()

        if self.current_reverse == 1:
            pattern_str.data = "RED"
        else:
            pattern_str.data = "WHITE"
        self.obj(pattern_str)
        self.past_reverse = self.current_reverse

if __name__ == "__main__":
    rospy.init_node("brake_lights_node")
    Brake_Lights()
    rospy.spin()

