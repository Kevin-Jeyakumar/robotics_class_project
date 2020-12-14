#! /usr/bin/env python3

import rospy
import actionlib
import lane_follow_isaac.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (ErrorAction) to the constructor.
    print("starting client")
    client = actionlib.SimpleActionClient('error_correction', lane_follow_isaac.msg.ErrorAction)

    print("waiting for server")
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    print("sending goal")
    # Creates a goal to send to the action server.
    goal = lane_follow_isaac.msg.ErrorGoal(error=10)

    # Sends the goal to the action server.
    client.send_goal(goal)

    print("waiting for result")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A ErrorResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
