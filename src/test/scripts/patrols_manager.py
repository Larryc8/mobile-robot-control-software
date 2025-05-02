#!/usr/bin/env python

import rospy
import actionlib
from actionlib_tutorials.msg import CountdownAction, CountdownGoal, CountdownResult, CountdownFeedback

class PatrolsServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('countdown', CountdownAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Countdown Action Server started")

    def execute(self, goal):
        rospy.loginfo("Received goal: countdown from %d", goal.target_number)
        
        # Check if the goal is valid
        if goal.target_number <= 0:
            result = CountdownResult()
            result.final_count = 0
            self.server.set_aborted(result, "Target number must be positive")
            return
        
        # Initialize feedback
        feedback = CountdownFeedback()
        
        # Perform the countdown
        for i in range(goal.target_number, -1, -1):
            # Check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("Countdown preempted")
                result = CountdownResult()
                result.final_count = i
                self.server.set_preempted(result)
                return
            
            # Publish feedback
            feedback.current_count = i
            self.server.publish_feedback(feedback)
            
            # Sleep for demonstration purposes
            rospy.sleep(1.0)
        
        # Return the result when done
        result = CountdownResult()
        result.final_count = 0
        self.server.set_succeeded(result, "Countdown completed successfully")

if __name__ == '__main__':
    rospy.init_node('countdown_server')
    server = PatrolsServer()
    rospy.spin()


