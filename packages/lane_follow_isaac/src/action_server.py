#! /usr/bin/env python3

import rospy
import actionlib
import lane_follow_isaac.msg

class ErrorAction(object):
    # create messages that are used to publish feedback/result
    _feedback = lane_follow_isaac.msg.ErrorFeedback()
    _result = lane_follow_isaac.msg.ErrorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, lane_follow_isaac.msg.ErrorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Lane lost, executing error correction from error %i' % (self._action_name, goal.error))
        
        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            break
        self._feedback.correction = goal.error / 2
        # publish the feedback
        self._as.publish_feedback(self._feedback)
          
        if success:
            self._result.correction = self._feedback.correction
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('error_correction')
    server = ErrorAction(rospy.get_name())
    rospy.spin()
