#!/usr/bin/env python

import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from pr2_controllers_msgs.msg import JointTrajectoryFeedback
from pr2_controllers_msgs.msg import JointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib import SimpleActionClient


class PR2JointTrajectoryControllerManager(object):
    def __init__(self, action_server, joint_names=[]):
        self.as_name = action_server
        self._ac = SimpleActionClient(self.as_name +
                                      '/joint_trajectory_action',
                                      JointTrajectoryAction)
        self.connect_as()

    def connect_as(self):
        rospy.loginfo("JointTrajectoryControllerManager: connecting to AS '" +
                      str(self.as_name) + "'")
        while not self._ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Waiting for AS '" + self.as_name + "'...")


if __name__ == '__main__':
    rospy.init_node('test_jtcm')
    jtcm = PR2JointTrajectoryControllerManager('/head_traj_controller')
