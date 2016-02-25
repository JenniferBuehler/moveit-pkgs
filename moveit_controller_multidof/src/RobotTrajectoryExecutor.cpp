#include <moveit_controller_multidof/RobotTrajectoryExecutor.h>
#include <moveit_controller_multidof/PathConverter.h>

// send robot path navigation and jaco arm trajectory in paralllel. If false, the path will be executed first.
#define SEND_TRAJECTORIES_PARALLEL true

using namespace moveit_controller_multidof;

RobotTrajectoryExecutor::RobotTrajectoryExecutor(const std::string& _virtual_joint_name,
        const std::string _trajectory_action_topic, const std::string _path_action_topic) : 
    trajectory_action_topic(_trajectory_action_topic),
    path_action_topic(_path_action_topic),
    has_path_navigator(false), 
    has_current_request(false),
    has_current_trajectory(false),
    last_exec(SUCCEEDED),
    virtual_joint_name(_virtual_joint_name),
    transform_path_running(false),
    trajectory_path_running(false) {

    ROS_INFO_STREAM("Loading RobotTrajectoryExecutor");

/*
    ros::NodeHandle node("~");
    
    node.param<std::string>("joint_trajectory_action_topic", trajectory_action_topic, DEFAULT_trajectory_action_topic);
    ROS_INFO("Got joint trajectory action topic name: <%s>", trajectory_action_topic.c_str());
    
    node.param<std::string>("path_navigation_action_topic", path_action_topic, "");
    ROS_INFO("Got path navigation action topic name: <%s>", path_action_topic.c_str());
  */
  
    has_path_navigator=(!virtual_joint_name.empty() && !path_action_topic.empty());

    if (virtual_joint_name.empty() != path_action_topic.empty())
    {
        ROS_WARN_STREAM("Specified either virtual joint or path action topic. "
            <<"But path navigation is not enabled unless both are set: "
            <<"Virtual joint name: "<<virtual_joint_name<<", path action topic: "<<path_action_topic);
    }

    joint_trajectory_action_client = new FollowJointTrajectoryActionClient(trajectory_action_topic, true);
    if (has_path_navigator) path_navigation_action_client = new PathNavigationActionClient(path_action_topic, true);

    /*if (!connectClients()) {
        ROS_ERROR("Not all clients connected");
    }*/

    ROS_INFO("RobotTrajectoryExecutor ready to go");
}


RobotTrajectoryExecutor::RobotTrajectoryExecutor(const RobotTrajectoryExecutor& other):
    trajectory_action_topic(other.trajectory_action_topic),
    path_action_topic(other.trajectory_action_topic),
    has_path_navigator(other.has_path_navigator), 
    has_current_request(other.has_current_request),
    has_current_trajectory(other.has_current_trajectory),
    last_exec(other.last_exec),
    virtual_joint_name(other.virtual_joint_name),
    current_trajectory(other.current_trajectory),
    transform_path_running(other.transform_path_running),
    trajectory_path_running(other.trajectory_path_running) {
    
    ROS_WARN("Using copy constructor of RobotTrajectoryExecutor");
    joint_trajectory_action_client = new FollowJointTrajectoryActionClient(trajectory_action_topic, true);
    if (has_path_navigator) path_navigation_action_client = new PathNavigationActionClient(path_action_topic, true);

    /*if (!connectClients()) {
        ROS_ERROR("Not all clients connected");
    }*/
} 


RobotTrajectoryExecutor::~RobotTrajectoryExecutor(){
    delete joint_trajectory_action_client;
    if (has_path_navigator) delete path_navigation_action_client;
}



bool RobotTrajectoryExecutor::sendTrajectory(const moveit_msgs::RobotTrajectory &t) {

    ROS_INFO("RobotTrajectoryExecutor: Received RobotTrajectory.");
    //ROS_INFO_STREAM(t);

    if (!clientsConnected()) {
        bool ret=connectClients();
        if (!connectClients()) {
            ROS_ERROR("RobotTrajecoryExecutor: Failed to connect to clients, hence can't execute RobotTrajectory");
            return false;
        }
    } 

    current_trajectory=t.joint_trajectory;

    bool execPathFirst=false;

    if (!t.multi_dof_joint_trajectory.points.empty())
    {
        ROS_INFO_STREAM("Getting trajectory "<<t.multi_dof_joint_trajectory);

        if (t.multi_dof_joint_trajectory.joint_names.size()!=1) {
            ROS_ERROR("Support only multiDOF joint trajectory of 1 joint, namely the virtual joint (of name %s)",
                virtual_joint_name.c_str());
            return false;
        }
        if (t.multi_dof_joint_trajectory.joint_names[0]!=virtual_joint_name) {
            ROS_ERROR("Support only multiDOF joint trajectory of virtual joint '%s', but got it for joint %s",
                virtual_joint_name.c_str(),t.multi_dof_joint_trajectory.joint_names[0].c_str());
            return false;
        }
    
        ROS_INFO("Got a joint trajectory of size %zu for virtual joint '%s'. NOTE: velocities and accellerations are not supported yet. File %s.",
            t.multi_dof_joint_trajectory.points.size(),virtual_joint_name.c_str(),__FILE__);    

        std::vector<geometry_msgs::Transform> transforms;
        std::vector<ros::Duration> times;
        for (int i=0; i<t.multi_dof_joint_trajectory.points.size(); ++i) {
            
            const std::vector<geometry_msgs::Transform>& transforms_=t.multi_dof_joint_trajectory.points[i].transforms; 
            if (transforms_.size()!=1) {
                ROS_ERROR("Consistency: Must have same number of transforms as joints, i.e. only 1 for the virtual joint");
            }

            geometry_msgs::Transform tr=transforms_[0];
        
            //ROS_INFO("Navigation to %f %f %f (quaternion %f %f %f)",tr.translation.x,tr.translation.y,tr.translation.z,
            //    tr.rotation.x,tr.rotation.y,tr.rotation.z);
            //ROS_INFO_STREAM("Time: "<<t.multi_dof_joint_trajectory.points[i].time_from_start);
            
            transforms.push_back(tr);    
            times.push_back(t.multi_dof_joint_trajectory.points[i].time_from_start);    
        }

    
        ROS_INFO("RobotTrajectoryExecutor: Sending navigation action request");
        //frame_id should be t.multi_dof_joint_trajectory.header.frame_id, but this is usually empty, so use the joint trajectories 
        if (!sendNavigationActionRequest(transforms,t.joint_trajectory.header.frame_id,-1,times)) {
            ROS_ERROR("could not send navigation request");
            return false;
        }
        execPathFirst=true;    
                
    }


    /*for (int i=0; i<t.joint_trajectory.points.size(); ++i) {
        ROS_INFO_STREAM("Joint trajectory point "<<i<<", time from start "<<t.joint_trajectory.points[i].time_from_start);
    }*/

    lock.lock();
    has_current_request=true;
    has_current_trajectory=true; 
    lock.unlock();

    last_exec = RUNNING;

    // send a goal to the action directly, as there is no path to be executed first
    if (SEND_TRAJECTORIES_PARALLEL || !execPathFirst) {
        if (!sendTrajectoryActionRequest(current_trajectory,-1)){
            ROS_ERROR("could not send navigation request");
            return false;
        }
    }
    return true;
}


bool RobotTrajectoryExecutor::cancelExecution() {     
    if (!clientsConnected()) {
        ROS_WARN("RobotTrajectoryExecutor: Canceling execution which can't have been successfully started before");
        return false;
    }
    
    lock.lock();
    if (has_current_request)
    {
        ROS_INFO_STREAM("RobotTrajectoryExecutor: Cancelling execution");
        last_exec = PREEMPTED;
        if (has_current_trajectory) {
            if (trajectory_path_running) joint_trajectory_action_client->cancelGoal();
        }
        if (has_path_navigator) {
            if (transform_path_running) path_navigation_action_client->cancelGoal();
        }
        has_current_request = false;
        has_current_trajectory = false;
    }
    lock.unlock();
    return true;
}

bool RobotTrajectoryExecutor::waitForExecution(const ros::Duration & timeout)
{
    if (!clientsConnected()) {
        ROS_WARN("RobotTrajectoryExecutor: Waiting for execution which can't have been successfully started before");
        return false;
    }
    
    lock.lock();
    bool active_request=has_current_request;
    bool active_path=transform_path_running;
    bool active_trajectory=trajectory_path_running;
    lock.unlock();

    // wait for the current execution to finish
    if (active_request) {
        //XXX this has to be improved to allow cancellation of goals if parallel execution is enabled and one fails at an early stage.

        ROS_INFO_STREAM("RobotTrajectoryExecutor: Waiting for execution for "<<timeout<<" secs");
        bool pathOk=!active_path || !has_path_navigator || path_navigation_action_client->waitForResult(timeout);
        
        if (!pathOk && SEND_TRAJECTORIES_PARALLEL && active_trajectory) {
            joint_trajectory_action_client->cancelGoal();
        }
        if (pathOk) {
            bool trajOk=!active_trajectory || joint_trajectory_action_client->waitForResult(timeout);
            if (trajOk) {
                ROS_INFO("RobotTrajectoryExecutor: Action succeeded.");
                last_exec = SUCCEEDED;
                return true;
            }
        }
        last_exec = TIMED_OUT;
        ROS_WARN_STREAM("RobotTrajectoryExecutor: Action timed out. Status: "<<last_exec);
    }
    return false;
}


void RobotTrajectoryExecutor::pathDoneCB(const actionlib::SimpleClientGoalState& state, const PathGoalResultConstPtr& result) {
    //ROS_INFO("Finished path in state [%s]", state.toString().c_str());
    
    setLastStateFrom(state);
        
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED) { //PENDING/ACTIVE/DONE
        ROS_WARN("Unsuccessful goal state detected, so not running the joint trajectory action request.");
        lock.lock();
        transform_path_running=false;
        lock.unlock();
        return;
    }    

    //ROS_INFO_STREAM("Answer: %i"<< result->finalpose);
    lock.lock();
    bool active_trajectory=has_current_trajectory;
    lock.unlock();

    if (!SEND_TRAJECTORIES_PARALLEL && active_trajectory) {
        if (!sendTrajectoryActionRequest(current_trajectory,-1)){
            ROS_ERROR("could not send navigation request");
        }
    }
}


void RobotTrajectoryExecutor::trajectoryDoneCB(const actionlib::SimpleClientGoalState& state,
    const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
    ROS_INFO("Finished joint trajectory in state [%s]", state.toString().c_str());
    //we're all done, so we can set the whole request as finished    
    lock.lock();
    has_current_trajectory=false;
    has_current_request=false;
    trajectory_path_running=false;
    lock.unlock();
    setLastStateFrom(state);
}



bool RobotTrajectoryExecutor::sendTrajectoryActionRequest(const trajectory_msgs::JointTrajectory& trajectory, float waitForResult) {
    if (!joint_trajectory_action_client->isServerConnected()) {
        ROS_ERROR_STREAM("RobotTrajectoryExecutor: Joint trajectory action client not connected: " << trajectory_action_topic);
        return false;
    }

    if (trajectory.joint_names.empty()) {
        //no trajectory to execute
        lock.lock();
        has_current_trajectory=false;
        has_current_request=false;
        lock.unlock();
        last_exec=SUCCEEDED;
        return true;
    }

    
    ROS_INFO("RobotTrajectoryExecutor Controller: Sending trajectory goal.");

    TrajectoryGoal tGoal;
    tGoal.trajectory = trajectory;

    joint_trajectory_action_client->sendGoal(tGoal, boost::bind(&RobotTrajectoryExecutor::trajectoryDoneCB, this, _1,_2));
    lock.lock();
    trajectory_path_running=true;
    lock.unlock();

    if (waitForResult < 0) return true;

    //wait for the action to return
    bool finished_before_timeout = joint_trajectory_action_client->waitForResult(ros::Duration(waitForResult));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = joint_trajectory_action_client->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
    } else {
        ROS_INFO_STREAM("Action did not finish before the time out. "<<joint_trajectory_action_client->getState().toString());
    }
    return false;
}




bool RobotTrajectoryExecutor::sendNavigationActionRequest(const std::vector<geometry_msgs::Transform>& transforms,
        const std::string& transforms_frame_id, const float waitForResult, 
        const std::vector<ros::Duration>& times) {

        bool transforms_are_relative=false;
        geometry_msgs::Pose start_pose=geometry_msgs::Pose(); //only plays a roe if transforms_are_relative=true
        
        nav_msgs::Path p;
        moveit_controller_multidof::TransformPathConverter::convert(transforms,p,transforms_frame_id,
            transforms_are_relative,start_pose,ros::Time::now(), times);

        ROS_INFO_STREAM("RobotTrajectoryExecutor: path after conversion: "<<p);

        return sendNavigationActionRequest(p,transforms_frame_id,waitForResult);    
}


bool RobotTrajectoryExecutor::sendNavigationActionRequest(const nav_msgs::Path& path,
    const std::string& path_frame_id, const float waitForResult)
{
    if (!has_path_navigator) {
        ROS_ERROR("Can't execute path, there is no path navigator configured");
    }

    if (!path_navigation_action_client->isServerConnected()) {
        ROS_ERROR_STREAM("RobotTrajectoryExecutor: Path execution action client not connected: " << path_action_topic);
        return false;
    }

    if (path.poses.empty()) {
        ROS_ERROR("Empty path, can't send navigation request");
        return false;    
    }

    ROS_INFO("RobotTrajectoryExecutor: Sending path goal.");
    
    PathGoal tGoal;
    tGoal.path.header.frame_id=path_frame_id;
    tGoal.path=path;
    
    //ROS_INFO_STREAM("Goal: "<<transform_goal);

    path_navigation_action_client->sendGoal(tGoal, boost::bind(&RobotTrajectoryExecutor::pathDoneCB, this, _1,_2));
    lock.lock();
    transform_path_running=true;
    lock.unlock();

    if (waitForResult < 0) return true;

    //wait for the action to return
    bool finished_before_timeout = path_navigation_action_client->waitForResult(ros::Duration(waitForResult));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = path_navigation_action_client->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
    } else {
        ROS_INFO_STREAM("Action did not finish before the time out. "<<path_navigation_action_client->getState().toString());
    }
    return false;
}


RobotTrajectoryExecutor::ExecutionStatus RobotTrajectoryExecutor::getLastExecutionStatus()
{
    return last_exec;
}

bool RobotTrajectoryExecutor::clientsConnected() {
    bool traj_connected=joint_trajectory_action_client->isServerConnected();    
    bool path_connected=false;
    if (has_path_navigator) path_connected=path_navigation_action_client->isServerConnected();    

    return traj_connected && (!has_path_navigator || path_connected);
}

bool RobotTrajectoryExecutor::connectClients() {
    unsigned int attempts = 0;

    bool traj_connected=joint_trajectory_action_client->isServerConnected();    
    bool path_connected=false;
    if (has_path_navigator) path_connected=path_navigation_action_client->isServerConnected();    

    while (ros::ok() && !traj_connected && (!has_path_navigator || !path_connected) && (++attempts < 3)){

        if (!traj_connected) {            
            ROS_INFO_STREAM("RobotTrajectoryExecutor: Waiting for " << trajectory_action_topic << " to come up");
            traj_connected=joint_trajectory_action_client->waitForServer(ros::Duration(2.0));
        }

        if (has_path_navigator && !path_connected) {
            ROS_INFO_STREAM("RobotTrajectoryExecutor: Waiting for " << path_action_topic << " to come up");
            path_connected=path_navigation_action_client->waitForServer(ros::Duration(2.0));
        }
    }

    bool ret=true;
    if (!joint_trajectory_action_client->isServerConnected()) {
        ROS_ERROR_STREAM("RobotTrajectoryExecutor: Joint trajectory action client not connected: " << trajectory_action_topic);
        ret=false;
    }
    
    if (has_path_navigator && !path_navigation_action_client->isServerConnected()) {
        ROS_ERROR_STREAM("RobotTrajectoryExecutor: Path execution action client not connected: " << path_action_topic);
        ret=false;
    }
    return ret;
}




void RobotTrajectoryExecutor::setLastStateFrom(const actionlib::SimpleClientGoalState& state) {
    if (state==actionlib::SimpleClientGoalState::SUCCEEDED) last_exec = SUCCEEDED;
    else if (state==actionlib::SimpleClientGoalState::ACTIVE) last_exec=RUNNING;
    else if (state==actionlib::SimpleClientGoalState::PREEMPTED) last_exec=PREEMPTED;
    else if (state==actionlib::SimpleClientGoalState::ABORTED) last_exec=ABORTED;
    else last_exec=UNKNOWN;
}



