This package provides a MoveIt! plugin to execute joint trajectories.
See also [this tutorial](https://github.com/JenniferBuehler/jb-ros-packs/wiki/Testing-Jaco-with-MoveIt)
for an example on how to use it with the Jaco arm.

This is essentially a variation of [moveit_plugins/moveit_simple_controller_manager](http://wiki.ros.org/moveit_simple_controller_manager),
only that this implementation supports only *one* type of controller: The "FollowRobotTrajectory" type.
Instead, it supports navigation of Multi-DOF trajectories in which (1) a **virtual joint** is
given and the trajectory given for it is the path for the robot to navigate (2) a regular
joint trajectory for the arm is given.

This implementation supports the same configuration as for
[moveit_plugins/moveit_simple_controller_manager](http://wiki.ros.org/moveit_simple_controller_manager),
but only can add controllers of type "FollowRobotTrajectory".

Accordingly, parameters in the .yaml config are to be in the following format:

```
virtual_joint_name: <name of virtual joint>
path_navigation_action_topic: /navigate_path
joint_trajectory_action_topic: /jaco/joint_trajectory_action
controller_list:
 - name: choose-any-name 
   action_ns: <not-currently-used> 
   type: FollowRobotTrajectory
   default: true
   joints:
     - virtual_joint
     - arm_joint_1
     - arm_joint_2
     - arm_joint_3
     - ...
```
Note that

* the type "FollowRobotTrajectory" has to be specified and
* the "virtual_joint_name" can be specified. If it is not specified, then the robot is considered immobile, and only
    joint trajectory actions are supported.
* The topic for the joint trajectory action needs to be specified, and
* optionally also specify the path navigation topic if robot is mobile.
* The list of joints must include all joints including the virtual joint.
* The parameter *action_ns* doesn't have to be specified as it is not used
  at the moment, but it should be declared because otherwise the parameters can't be
  read uniformly with the code of MoveItSimpleControllerManager.
