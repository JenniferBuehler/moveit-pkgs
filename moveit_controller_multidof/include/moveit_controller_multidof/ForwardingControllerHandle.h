#ifndef MOVEIT_CONTROLLER_MULTIDOF_FORWARDINGCONTROLLERMANAGER_H
#define MOVEIT_CONTROLLER_MULTIDOF_FORWARDINGCONTROLLERMANAGER_H

#include <map>

#include <ros/ros.h>

#include <moveit/controller_manager/controller_manager.h>
#include <moveit_controller_multidof/RobotTrajectoryExecutor.h>

namespace moveit_controller_multidof 
{

MOVEIT_CLASS_FORWARD(ForwardingControllerHandle);

/** 
 * \brief Uses a moveit_controller_multidof::RobotTrajectoryExecutor to execute a trajectory.
 * 
 * \author Jennifer Buehler
 * \date February 2016
 */
class ForwardingControllerHandle: public ActionBasedControllerJointsHandle
{
public:

    ForwardingControllerHandle(const std::string &joint_action_topic,
        const std::string& path_action_topic,
        const std::string& virtual_joint_name) :
        ActionBasedControllerJointsHandle("no-name"),
        trajectory_executor(virtual_joint_name, joint_action_topic, path_action_topic),
        last_exec(moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
    }


    virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &t) {
        return trajectory_executor.sendTrajectory(t);
    }

    virtual bool cancelExecution() {    
        return trajectory_executor.cancelExecution(); 
    }
    
    virtual bool waitForExecution(const ros::Duration & timeout)
    {
        return trajectory_executor.waitForExecution(timeout);
    }
    
    virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
    {
        moveit_controller_multidof::RobotTrajectoryExecutor::ExecutionStatus stat=trajectory_executor.getLastExecutionStatus();
        moveit_controller_manager::ExecutionStatus exec=moveit_controller_manager::ExecutionStatus::UNKNOWN;
        if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::RUNNING) exec=moveit_controller_manager::ExecutionStatus::RUNNING;
        else if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::SUCCEEDED) exec=moveit_controller_manager::ExecutionStatus::SUCCEEDED;
        else if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::PREEMPTED) exec=moveit_controller_manager::ExecutionStatus::PREEMPTED;
        else if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::TIMED_OUT) exec=moveit_controller_manager::ExecutionStatus::TIMED_OUT;
        else if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::ABORTED) exec=moveit_controller_manager::ExecutionStatus::ABORTED;
        else if (stat==moveit_controller_multidof::RobotTrajectoryExecutor::FAILED) exec=moveit_controller_manager::ExecutionStatus::FAILED;
        return exec;
    }
private:

    RobotTrajectoryExecutor trajectory_executor;    
    moveit_controller_manager::ExecutionStatus last_exec;

};



} // end namespace 


#endif  // MOVEIT_CONTROLLER_MULTIDOF_FORWARDINGCONTROLLERMANAGER_H
