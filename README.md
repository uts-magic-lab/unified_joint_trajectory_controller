# unified_joint_trajectory_controller
Package to unify all the controllers of a robot for the different joints
to be able to be controlled from a single point in user friendly ways.

The focus is to ease the use of ROS robots and ease the construction of
tools to move the robot (GUIs, web based GUI, controllers, apps...).

**Work in progress.**

# Aims of the package

* Create just a `yaml` config file to describe the controllers of the robot.
```xml
unified_joint_trajectory_controller:
  - controller_type: joint_trajectory_controller
    action_server: '/head_traj_controller'
    joint_names: ['head_pan_joint', 'head_tilt_joint']

  - controller_type: joint_trajectory_controller
    action_server: '/r_arm_controller'
    joint_names: ['r_shoulder_pan_joint', 'r_upper_arm_roll_joint',
                  'r_shoulder_lift_joint', 'r_forearm_roll_joint',
                  'r_elbow_flex_joint', 'r_wrist_flex_joint',
                  'r_wrist_roll_joint',]
  ...
```
* Provide easy to use interfaces to move any number of joints pertaining to any group
of the robot. Action server of FollowJointTrajectory, topic of JointTrajectory. 
* Provide even easier to use interfaces to move any number of joints to a pose, action server and topic to achieve just poses with joints, positions and final time in a simple message.
```
string[] joint_names
float64[] positions
duration time_from_start
```
* Allow sending incremental joint positions in the same fashion than trajectories and poses explained before.
* Allow execution of multiple trajectories at the same time if they don't collide
in resources. (Blending of trajectories should be done in a C++ implementation which isn't 
in the plans.)
* Allow cancel of trajectories.
* Expose joints, joint limits, controllable and uncontrollable joints list in a user friendly way (param server) for ease of use from third party apps.

# Not in the roadmap
This package does **not** aim to offer safety on the trajectories poses altho 
if needed I'll implement one based on this package and MoveIt!.

# Interfaces to be supported
* joint_trajectory_controller (PR2)
* follow_joint_trajectory
* pr2_gripper_action
* simple-one-topic-per-motor interface

If I find out a robot with a different interface I need to use I'll also implement it.

# Extra command-line (and ease to use classes) tools

* `./command_joints.py joint1 joint2 ... jointN pos1 pos2 ... posN time`
* `./get_current_joint_pos.py joint1 joint2 ... jointN`
* `./get_joint_limits.py [joint1 joint2 ... jointN]`
