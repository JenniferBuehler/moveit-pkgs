#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <path_navigation_msgs/TransformPathExecutionAction.h>
#include <path_navigation_msgs/TransformPathExecutionActionResult.h>
#include <path_navigation_msgs/PathExecutionAction.h>
#include <path_navigation_msgs/PathExecutionActionResult.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <moveit_msgs/RobotTrajectory.h>

#define DEFAULT_TRAJECTORY_ACTION_TOPIC "/joint_trajectory_action"

namespace moveit_controller_multidof  {

/** 
 * Executes a moveit_msgs/RobotTrajectory and takes the MultiDOF information from the virtual joint
 * to make the robot navigate to that location.
 *
 * So far, multi DOF trajectories (the virtual joint trajectories) are supported in the following way:
 * Only 1 virtual joint is allowed, which specifies the robot navigation path. It is not executedin parallel to the
 * arm's JointTrajectory though, as it probably is intended to. The arm's joint trajectory is only executed AFTER
 * the robot has navigated along the path. This has to be changed later, a simple solution would be to take the 
 * trajectories apart into single points and send them in parallel to navigation and arm movement.
 *
 * \author Jennifer Buehler
 */
class RobotTrajectoryExecutor {
public:
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionClient;
    typedef actionlib::SimpleActionClient<path_navigation_msgs::PathExecutionAction> PathNavigationActionClient;
    typedef typename path_navigation_msgs::PathExecutionGoal PathGoal;
    typedef typename path_navigation_msgs::PathExecutionResult PathGoalResult;
    typedef typename path_navigation_msgs::PathExecutionResultConstPtr PathGoalResultConstPtr;
    
    typedef typename control_msgs::FollowJointTrajectoryGoal TrajectoryGoal;
    typedef typename control_msgs::FollowJointTrajectoryResult TrajectoryGoalResult;

    //\brief Defines the various states the execution of the aciton can be in
    enum ExecStatus {
        RUNNING, SUCCEEDED, PREEMPTED, TIMED_OUT, ABORTED, FAILED, UNKNOWN
    };
    typedef ExecStatus ExecutionStatus;

    /**
     * \param _virtual_joint_name name of the virtual joint. If empty,
     * then only execution of joint trajectories (no navigation) will be supported.
     * \param _trajectory_action_topic the action topic onto which to forward the joint trajectory
     * \param _path_action_topic the action topic onto which to forward the path navigation. This can be empty
     *  if \e _virtual_joint_name is empty as well and the robot is not mobile.
     */
    RobotTrajectoryExecutor(const std::string& _virtual_joint_name,
        const std::string _trajectory_action_topic,
        const std::string _path_action_topic); 

    ~RobotTrajectoryExecutor();
    bool sendTrajectory(const moveit_msgs::RobotTrajectory &t);
    
    bool cancelExecution();
    
    bool waitForExecution(const ros::Duration & timeout);

    ExecutionStatus getLastExecutionStatus();

private:
    RobotTrajectoryExecutor(const RobotTrajectoryExecutor& other);

    bool connectClients();
    bool clientsConnected();

    // Called once navigation action completes
    void pathDoneCB(const actionlib::SimpleClientGoalState& state, const PathGoalResultConstPtr& result);

    // Called once trajectory action completes
    void trajectoryDoneCB(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    //\param waitForResult if negative, the action does not wait for the result. If positive, this is the number of seconds
    //it will wait for the action to finish.
    bool sendTrajectoryActionRequest(const trajectory_msgs::JointTrajectory& trajectory, float waitForResult=-1);

    bool sendNavigationActionRequest(const std::vector<geometry_msgs::Transform>& transforms,
        const std::string& transforms_frame_id, const float waitForResult, 
        const std::vector<ros::Duration>& times=std::vector<ros::Duration>());

    //\param waitForResult if negative, the action does not wait for the result. If positive, this is the number of seconds
    //it will wait for the action to finish.
    bool sendNavigationActionRequest(const nav_msgs::Path& p,
        const std::string& transforms_frame_id,
        const float waitForResult=-1);

    //sets last executions status (last_exec) from this simple client goal state
    void setLastStateFrom(const actionlib::SimpleClientGoalState& state);

    std::string trajectory_action_topic;
    std::string path_action_topic;

    FollowJointTrajectoryActionClient * joint_trajectory_action_client;
    PathNavigationActionClient * path_navigation_action_client;
    
    bool has_path_navigator;
    
    bool has_current_request;
    bool has_current_trajectory;

    trajectory_msgs::JointTrajectory current_trajectory;

    bool transform_path_running;        
    bool trajectory_path_running;        
    
    //protects, has_current_request, has_current_trajectory and transform_path_running, trajectory_path_running 
    boost::mutex lock;
    
    ExecutionStatus last_exec;
     
    std::string virtual_joint_name;
};

} // end namespace 
