#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, LanePose
from PID import PID_Controller

class Lab3:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.sub = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)

        self.pose = LanePose()
        self.speed = Twist2DStamped()

        self.pid_d = PID_Controller()
        self.pid_d.set_params(2, 0.2, 0.01)
        self.pid_phi = PID_Controller()
        self.pid_d.set_params(-8, 0.2, 0.01)

    def callback(self, data):
        rospy.logwarn("LAB3 CALLBACK")
        control_d = self.pid_d.get_acc(data.d, rospy.get_time())
        control_phi = self.pid_phi.get_acc(data.phi, rospy.get_time())

        self.speed.v = 0.4
        self.speed.omega = control_d + control_phi - 0.25
        rospy.logwarn("OMEGA = %f", self.speed.omega)
        rospy.loginfo(self.speed)
        self.pub.publish(self.speed)

    def stop(self):
        self.speed.v = 0.0
        self.speed.omega = 0.0
        rospy.loginfo(self.speed)
        self.pub.publish(self.speed)


if __name__ == '__main__':
    try:
        rospy.logwarn("STARTING LAB3 NODE")
        rospy.init_node('lanefollow_isaac', anonymous=True)
        c = Lab3()

        rate = rospy.Rate(5) # 5hz
        while(1):
            if rospy.has_param("/kp_d"):
                c.pid_d.set_params(rospy.get_param("/kp_d"), 0, 0)
            if rospy.has_param("/kp_phi"):
                c.pid_phi.set_params(rospy.get_param("/kp_phi"), 0, 0)
            rate.sleep()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass