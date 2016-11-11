#!/usr/bin/env python

import rospy
# Import controller manager classes
from unified_joint_trajectory_controller.pr2_joint_trajectory_controller_manager import PR2JointTrajectoryControllerManager


class UnifiedInterface(object):
    def __init__(self):
        rospy.loginfo("Initializing UnifiedInterface...")
        self.configure()
        self.joints_to_control = []
        self.controllers = []

        # Advertise AS

        # Advertise topic

        # Advertise incremental AS

        # Advertise incremental topic

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
        if c_type == 'joint_trajectory_controller':
            action_server_name = controller_dict.get('action_server')
            joint_names = controller_dict.get('joint_names')
            if joint_names is None:
                self.get_joint_names_from_state(action_server_name)

        elif c_type == 'follow_joint_trajectory_controller':
            #

        elif c_type is None:
            rospy.lowarn("A controller configuration returned None on " +
                         " params config: " + str(controller_dict))

    def get_joint_names_from_state(self, server_name):
        rospy.wait_for_message(server_name + '/state', )

if __name__ == '__main__':
    rospy.init_node('unified_interface')
    ui = UnifiedInterface()
    rospy.spin()
