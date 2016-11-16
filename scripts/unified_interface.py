#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryFeedback
from actionlib import ActionServer
# This package messages
from unified_joint_trajectory_controller.msg import GoJointsToPositionAction
from unified_joint_trajectory_controller.msg import GoJointsToPositionGoal
from unified_joint_trajectory_controller.msg import GoJointsToPositionResult
from unified_joint_trajectory_controller.msg import GoJointsToPositionFeedback
from unified_joint_trajectory_controller.msg import JointsToPosition


class UnifiedInterface(object):
    def __init__(self):
        rospy.loginfo("Initializing UnifiedInterface...")
        self.joints_to_control = []
        self.non_controlled_joints = []
        self.controllers = []
        self.configure()

        # Advertise ASs
        self.ns = rospy.get_name()
        self._as = ActionServer(self.ns + '/follow_joint_trajectory',
                                          FollowJointTrajectoryAction,
                                          goal_cb=self.exec_cb,
                                          auto_start=False)
        self._as.start()

        self._as_simple = ActionServer(self.ns + '/joints_to_pose',
                                       GoJointsToPositionAction,
                                       goal_cb=self.simple_exec_cb,
                                       auto_start=False)

        # Advertise topics
        self._sub = rospy.Subscriber(self.ns + '/command',
                                     JointTrajectory,
                                     self.topic_cb,
                                     queue_size=1)

        self._simple_sub = rospy.Subscriber(self.ns + '/joints_to_pose_topic',
                                            JointsToPosition,
                                            self.simple_topic_cb,
                                            queue_size=1)

        # Advertise incremental ASs
        self._as_incr = ActionServer(self.ns + '/incremental_follow_joint_trajectory',
                                               FollowJointTrajectoryAction,
                                               goal_cb=self.exec_incr_cb,
                                               auto_start=False)
        self._as_simple_incr = ActionServer(self.ns + '/incremental_joints_to_pose',
                                            GoJointsToPositionAction,
                                            goal_cb=self.simple_exec_incr_cb,
                                            auto_start=False)

        # Advertise incremental topics
        self._sub_incr = rospy.Subscriber(self.ns + '/incr_command',
                                          JointTrajectory,
                                          self.topic_cb,
                                          queue_size=1)

        self._simple_sub_incr = rospy.Subscriber(self.ns + '/incr_joints_to_pose_topic',
                                                 JointsToPosition,
                                                 self.simple_topic_cb,
                                                 queue_size=1)

        # Subscribe to joint states if necessary?

    def configure(self):
        """Configure the node from the param server"""
        # Read config from param server
        node_cfg = rospy.get_param(
            '/unified_joint_trajectory_controller', None)
        if node_cfg is None:
            rospy.logerr("No '/unified_joint_trajectory_controller'" +
                         " param found!")
            exit(0)

        # Parse every block
        controllers = node_cfg.get('controllers')
        if controllers is None:
            rospy.logerr("No controllers in" +
                         "'/unified_joint_trajectory_controller/controllers'," +
                         " nothing to control")

        # Controllers should be a list of dictionaries
        # containing 'controller_type', from there
        # we deduct what other keys are needed
        for c in controllers:
            current_c = self.create_controller(c)
            self.controllers.append(current_c)

    def create_controller(self, controller_dict):
        c_type = controller_dict.get('controller_type')
        if c_type == 'pr2_follow_joint_trajectory_controller':
            # Import dynamically what we need
            from unified_joint_trajectory_controller.pr2_joint_trajectory_controller_manager import PR2JointTrajectoryControllerManager
            action_server_name = controller_dict.get('action_server')
            return PR2JointTrajectoryControllerManager(action_server_name)

        elif c_type == 'follow_joint_trajectory_controller':
            # TODO:
            return None

        elif c_type is None:
            rospy.lowarn("A controller configuration returned None on " +
                         " params config: " + str(controller_dict))
            return None

    def send_goal(self, goal):
        # send goals to all controllers
        for controller in self.controllers:
            controller.send_goal(goal)

        # publish feedback until they are done
        done_goals = [False] * len(self.controllers)
        while not rospy.is_shutdown() and not all(done_goals):
            # construct feedback message
            curr_fed = FollowJointTrajectoryFeedback()
            for cidx, controller in enumerate(self.controllers):
                if controller.is_goal_done():
                    done_goals[cidx] = True
                # add to feedback message this controller stuff
                fed = controller.get_feedback()
                # curr_fed add stuff

        # compose global result and publish
        for controller in self.controllers:
            res = controller.get_result()

    def cancel_all_goals(self):
        for c in self.controllers:
            c.cancel_all_goals()

    def exec_cb(self, goal):
        goal.set_accepted()
        rospy.loginfo("Goal is: " + str(goal.get_goal()))
        self.send_goal(goal.get_goal().trajectory)

    def simple_exec_cb(self, goal):
        pass

    def topic_cb(self, goal):
        pass

    def simple_topic_cb(self, goal):
        pass

    def exec_incr_cb(self, goal):
        pass

    def simple_exec_incr_cb(self, goal):
        pass

    def topic_incr_cb(self, goal):
        pass

    def simple_topic_incr_cb(self, goal):
        pass


if __name__ == '__main__':
    rospy.init_node('unified_interface')
    ui = UnifiedInterface()
    rospy.loginfo("Ready.")
    rospy.spin()
