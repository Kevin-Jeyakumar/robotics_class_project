#!/usr/bin/env python3

import rospy
import actionlib
import lane_follow_isaac.msg
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, LanePose
from PID import PID_Controller

class Lab3:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.sub = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.client = actionlib.SimpleActionClient('error_correction', lane_follow_isaac.msg.ErrorAction)

        self.pose = LanePose()
        self.speed = Twist2DStamped()

        self.pid_d = PID_Controller()
        self.pid_d.set_params(2, 0.2, 0.01)
        self.pid_phi = PID_Controller()
        self.pid_d.set_params(-8, 0.2, 0.01)

        self.rate = rospy.Rate(5)

    def callback(self, data):
        rospy.logwarn("DATA_PHI = %f", data.phi)
        control_d = self.pid_d.get_acc(data.d, rospy.get_time())
        control_phi = self.pid_phi.get_acc(data.phi, rospy.get_time())

        if abs(data.phi) > 1:
            # Send request to action server
            self.client.wait_for_server()
            goal = lane_follow_isaac.msg.ErrorGoal(error=data.phi)
            self.client.send_goal(goal)

            # Reverse in preparation
            for i in range(5):
                self.stop()
            for i in range(5):
                self.reverse()
            for i in range(5):
                self.stop()

            # Get correction result from action server
            self.client.wait_for_result()
            res = self.client.get_result()  # A ErrorResult

            self.pid_d.reset_controller()
            self.pid_phi.reset_controller()

            for i in range(5):
                self.turn(res.correction)
            for i in range(5):
                self.stop()

            self.pid_d.reset_controller()
            self.pid_phi.reset_controller()

        else:
            self.speed.v = 0.4
            self.speed.omega = control_d + control_phi - 0.25
            rospy.loginfo(self.speed)
            self.pub.publish(self.speed)

    def stop(self):
        self.speed.v = 0.0
        self.speed.omega = 0.0
        rospy.loginfo(self.speed)
        self.pub.publish(self.speed)

    def reverse(self):
        self.speed.v = -0.4
        self.speed.omega = 0.0
        rospy.loginfo(self.speed)
        self.pub.publish(self.speed)

    def turn(self, data):
        control_phi = self.pid_phi.get_acc(data, rospy.get_time())
        
        self.speed.v = 0.0
        self.speed.omega = control_phi
        rospy.loginfo(self.speed)
        self.pub.publish(self.speed)


if __name__ == '__main__':
    try:
        rospy.logwarn("STARTING LAB3 NODE")
        rospy.init_node('lane_follow', anonymous=True)
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
