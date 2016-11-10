#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer

"""
Create a fake JointTrajectory or FollowJointTrajectory
action server to test stuff.

Author: Sammy Pfeiffer <Sammy.Pfeiffer@student.uts.edu.au>
"""


class FakeJointTrajController(object):
    def __init__(self, action_server, has_state=True, has_command=True,
                 use_pr2_trajectory=True,
                 joint_names=[]):
        self.joint_names = joint_names
        if use_pr2_trajectory:
            from pr2_controllers_msgs.msg import JointTrajectoryAction as action_def
            from pr2_controllers_msgs.msg import JointTrajectoryGoal as goal_def
            from pr2_controllers_msgs.msg import JointTrajectoryFeedback as feedback_def
            from pr2_controllers_msgs.msg import JointTrajectoryResult as result_def
            action = '/joint_trajectory_action'
        else:
            from control_msgs.msg import FollowJointTrajectoryAction as action_def
            from control_msgs.msg import FollowJointTrajectoryGoal as goal_def
            from control_msgs.msg import FollowJointTrajectoryFeedback as feedback_def
            from control_msgs.msg import FollowJointTrajectoryResult as result_def
            action = '/follow_joint_trajectory'
        self.feedback_def = feedback_def
        self.result_def = result_def
        self.as_name = action_server
        rospy.loginfo("Action server listening at: " + self.as_name + action)

        self._as = SimpleActionServer(self.as_name + action,
                                      action_def,
                                      execute_cb=self.cb,
                                      auto_start=False)
        self._as.start()

        # Has /server_name/state topic
        if has_state:
            if use_pr2_trajectory:
                from pr2_controllers_msgs.msg import JointTrajectoryControllerState
            else:
                from control_msgs.msg import JointTrajectoryControllerState
            self.last_state = JointTrajectoryControllerState()
            rospy.loginfo("Publishing state at: " + self.as_name + '/state')
            self.state_pub = rospy.Publisher(self.as_name + '/state',
                                             JointTrajectoryControllerState,
                                             queue_size=1)
            self.timer = rospy.Timer(rospy.Duration(0.1), self.state_timer_cb)

        # Has /server_name/command topic
        if has_command:
            from trajectory_msgs.msg import JointTrajectory
            rospy.loginfo("Listening to commands at: " +
                          self.as_name + '/command')
            self.cmd_sub = rospy.Subscriber(self.as_name + '/command',
                                            JointTrajectory,
                                            self.cmd_cb,
                                            queue_size=1)

    def state_timer_cb(self, timer_event):
        state = self.last_state
        state.header.stamp = rospy.Time.now()
        state.joint_names = self.joint_names
        if len(joint_names) > 0 and len(state.desired.positions) == 0:
            for j in joint_names:
                state.desired.positions.append(0.0)
                state.desired.velocities.append(0.0)
                state.desired.time_from_start = rospy.Duration(0.0)
                state.actual = state.desired
                state.error = state.desired
        self.state_pub.publish(state)

    def cmd_cb(self, msg):
        rospy.loginfo("Got command!")

    def cb(self, goal):
        # Do something with the goal, publish one feedback
        # and then result, always succeed
        rospy.loginfo("Got goal!")
        self._as.publish_feedback(self.feedback_def())
        rospy.sleep(0.1)
        self._as.set_succeeded(self.result_def())


if __name__ == '__main__':
    rospy.init_node('fake_joint_traj_cnt', anonymous=True)
    argv = rospy.myargv()

    if len(argv) < 2 or "--help" in argv or "-h" in argv:
        print "Usage:"
        print argv[0] + " action_server_name [has_state] [has_command] [use_pr2_trajectory] joint_names j1 j2 ... jN"
        print
        print "Example:"
        print argv[0] + " /head_traj_controller has_state has_command use_pr2_trajectory joint_names head_pan_joint head_tilt_joint"
        exit(0)

    action_server_name = argv[1]

    has_state = False
    if 'has_state' in argv:
        has_state = True

    has_command = False
    if 'has_command' in argv:
        has_command = True

    use_pr2_trajectory = False
    if 'use_pr2_trajectory' in argv:
        use_pr2_trajectory = True

    if 'joint_names' in argv:
        idx = argv.index('joint_names')
        joint_names = argv[idx + 1:]
    else:
        joint_names = []

    fjtc = FakeJointTrajController(action_server_name, has_state,
                                   has_command, use_pr2_trajectory,
                                   joint_names)
    rospy.spin()
