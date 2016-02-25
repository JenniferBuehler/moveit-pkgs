#ifndef MOVEIT_CONTROLLER_MULTIDOF_MULTIDOFCONTROLLERMANAGER_H
#define MOVEIT_CONTROLLER_MULTIDOF_MULTIDOFCONTROLLERMANAGER_H

/*
XXX MULTIDOF_CHANGE: modified includes
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <pluginlib/class_list_macros.h>
*/

#include <ros/ros.h>
#include <algorithm>
#include <map>
#include <moveit/controller_manager/controller_manager.h>
// XXX END MULTIDOF_CHANGE


// XXX MULTIDOF_CHANGE: this include file now ONLY contains ActionBasedControllerHandleBase 
#include <moveit_controller_multidof/action_based_controller_handle.h>

namespace moveit_controller_multidof 
{

/**
 * Essentially a variation of MoveItSimpleControllerManager
 * (moveit_plugins/moveit_simple_controller_manager)
 * which supports only *one* type of controller: The "FollowRobotTrajectory" type.
 *
 * The "FollowRobotTrajectory" controller does nothing but take the
 * moveit_msgs/RobotTrajectory sent by MoveIt! and *forward* it to 
 * an implementation which follows the robot trajectory *including* taking
 * into account navigation for the virtual joint, specified in
 * moveit_msgs::RobotTrajectory.multi_dof_joint_trajectory.
 *
 * The actual following of the moveit_msgs/RobotTrajectory
 * is provided in the moveit_controller_multidof::ForwardingControllerHandle.
 * 
 * This implementation is essentially a copy of MoveItSimpleControllerManager
 * (moveit_plugins/moveit_simple_controller_manager, with the only change 
 * that only the "FollowRobotTrajectory" type is supported.
 * It would be nice to *extend* MoveItSimpleControllerManager
 * to support the type of controller which has a concept of the virtual joint
 * and therefore supports MulitDOF trajectories. However unfortunately at this stage
 * the implementation of MoveItSimpleControllerManager is in a .cpp file and can't
 * be extended. Some time in future this may be the case, and then this implementation
 * should be changed accordingly. For now, the implementation is made to act just as
 * an exteded version of MoveItSimpleControllerManager.
 *
 * Accordingly, parameters in the .yaml config are to be in the following format:
 *
 * ```
 * virtual_joint_name: <name of virtual joint>
 * path_navigation_action_topic: /navigate_path
 * joint_trajectory_action_topic: /jaco/joint_trajectory_action
 * controller_list:
 *  - name: choose-any-name 
 *    action_ns: <not-currently-used> 
 *    type: FollowRobotTrajectory
 *    default: true
 *    joints:
 *      - virtual_joint
 *      - arm_joint_1
 *      - arm_joint_2
 *      - arm_joint_3
 *      - ...
 * ```
 *
 * Note that the type "FollowRobotTrajectory" is new, and that "virtual_joint_name" can
 * be specified. If it is not specified, then the robot is considered immobile, and only
 * joint trajectory actions are supported. The topic for the joint trajectory action
 * needs to be specified, and also the path navigation topic if robot is mobile.
 * The list of joints must include all joints including the virtual joint.
 * The parameter action_ns doesn't have to be specified as it is not used
 * at the moment, but it should be declared because otherwise the parameters can't be
 * read uniformly with the code of MoveItSimpleControllerManager.
 * 
 * *Note for developers*: All changes to the copy of MoveItSimpleControllerManager that have
 * been made are in-between comments "MULTIDOF_CHANGE". This should make
 * updating this implementation according to newer versions of MoveItSimpleControllerManager
 * easier. Unfortunately at this stage, this is the easiest way to do this.
 *  
 *
 * \author Jennifer Buehler
 * \date February 2016
 */
class MultiDOFControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  MultiDOFControllerManager();
  virtual ~MultiDOFControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name);

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names);

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints); 

  /*
   * Controllers are all active and default -- that's what makes this thing simple.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name);

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }

protected:

  ros::NodeHandle node_handle_;
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

} // end namespace moveit_simple_controller_manager


#endif  // MOVEIT_CONTROLLER_MULTIDOF_MULTIDOFCONTROLLERMANAGER_H
