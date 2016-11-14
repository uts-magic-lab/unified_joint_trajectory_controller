#!/usr/bin/env python

# Standard ROS imports
import rospy
from actionlib import SimpleActionClient
# Messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# This specific controller manager needed messages
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from pr2_controllers_msgs.msg import JointTrajectoryFeedback
from pr2_controllers_msgs.msg import JointTrajectoryResult
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

"""
Class to manage the connection to a controller
of type pr2_controllers to keep track of the state
and send trajectories to the joint_trajectory_action
interface.

Author: Sammy Pfeiffer
"""


class PR2JointTrajectoryControllerManager(object):
    def __init__(self, namespace):
        """
        Setup connection to ``namespace/joint_trajectory_action`` action server
        and ``namespace/state`` state topic.

        :param str namespace: namespace for the controller from where
            the action server and the state topic hang, e.g. /head_controller
            /head_controller/joint_trajectory_action/[goal/feedback/cancel/result]
            /head_controller/state
        """
        self.ns = namespace
        self.goal_done = False
        self.last_state = None
        self.state_topic = self.ns + '/state'
        self._state_sub = rospy.Subscriber(self.state_topic,
                                           JointTrajectoryControllerState,
                                           self.state_cb,
                                           # Only the last state is important
                                           queue_size=1)

        self.as_name = self.ns + '/joint_trajectory_action'
        self._ac = SimpleActionClient(self.as_name,
                                      JointTrajectoryAction)
        self.connect_as()

        self.wait_for_state()
        self.managed_joints = self.last_state.joint_names

    def connect_as(self):
        """
        Connect to the action server printing warnings while
        it's waiting.
        """
        rospy.loginfo("JointTrajectoryControllerManager: connecting to AS '" +
                      str(self.as_name) + "'")
        while not self._ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Waiting for AS '" + self.as_name + "'...")

    def wait_for_state(self):
        """
        Wait for the first state message.
        """
        rospy.loginfo("Waiting for first state on: '" +
                      self.state_topic + "'...")
        while not rospy.is_shutdown() and self._last_state is None:
            rospy.sleep(2.0)
            rospy.logwarn("Waiting for first state on: '" +
                          self._ns + "/state'...")

    def state_cb(self, state):
        """
        Save last state received.

        :param JointTrajectoryControllerState state: last state.
        """
        self.last_state = state

    def send_goal(self, goal):
        """
        Send goal to the controller.

        :param JointTrajectory goal: Goal to send.
        """
        self.goal_done = False
        self.last_result = None
        self.last_feedback = None
        # Check which of our joints the goal has
        joints_in_goal = []
        for j in goal.joint_names:
            if j in self.managed_joints:
                joints_in_goal.append(j)

        missing_joints = list(set(self.managed_joints) - set(joints_in_goal))
        if len(missing_joints) == len(self.managed_joints):
            # This goal contains no joints of this controller
            # we don't do anything
            rospy.loginfo("Goal contains no joints of this controller.")
            return

        if len(missing_joints) > 0:
            # Fill message for the real controller with the missing joints
            # Transform tuple fields into lists
            goal.joint_names = list(goal.joint_names)
            for point in goal.points:  # type: JointTrajectoryPoint
                point.positions = list(point.positions)
                point.velocities = list(point.velocities)
                point.accelerations = list(point.accelerations)
                point.effort = list(point.effort)

            # Now add the missing joints
            for jname in missing_joints:
                goal.joints_names.append(jname)
                jidx = self.managed_joints.index(jname)
                # TODO: remove this if-assert (it's redundant)
                if jidx == -1:
                    rospy.logerror("Couldn't find joint " + jname +
                                   ", which we should be controlling," +
                                   " in our managed joints.")
                    assert False
                for point in goal.points:
                    point.positions.append(
                        self.last_state.actual.positions[jidx])
                    point.velocities.append(
                        self.last_state.actual.velocities[jidx])
                    point.accelerations.append(
                        self.last_state.actual.accelerations[jidx])
                    # TODO: deal with forces, they aren't published in state...
                    # but they are in joint_states

        # Now the trajectory should be complete and we can send the goal
        jtg = JointTrajectoryGoal()
        jtg.trajectory = goal
        self._ac.send_goal(jtg,
                           done_cb=self.done_cb,
                           feedback_cb=self.feedback_cb)

    def get_managed_joints(self):
        """
        Convenience method, return list of managed joints.

        :returns list: list of strings of managed joints.
        """
        return self.managed_joints

    def is_goal_done(self):
        """
        Convenience method, return true if the last goal is done.

        :returns bool: True if goal done, False otherwise.
        """
        return self.goal_done

    def get_result(self):
        """
        Convenience method, returns result.

        :returns JointTrajectoryResult: result.
        """
        return self.last_result

    def done_cb(self, state, result):
        """
        :param int state: int as state actionlib_msgs/GoalStatus.
        :param JointTrajectoryResult result: result of the action server.
        """
        self.goal_done = True
        self.last_result = result

    def get_feedback(self):
        """
        Convenience method, get feedback.

        :returns JointTrajectoryFeedback: feedback.
        """
        return self.last_feedback

    def feedback_cb(self, feedback):
        """
        :param JointTrajectoryFeedback feedback: feedback message.
        """
        self.last_feedback = feedback

    def cancel_all_goals(self):
        """
        Cancel all current goals if any.
        """
        self._ac.cancel_all_goals()

# TODO: Add checking if action server is still alive
# and manage reconnection (I think this is not automatic)


if __name__ == '__main__':
    rospy.init_node('test_jtcm')
    jtcm = PR2JointTrajectoryControllerManager('/head_traj_controller')
    # Try to send a goal with all joints

    # Try to send a goal with extra joints that dont pertain

    # Try to send a goal missing joints

    # Try to send a goal with no joints that pertain

