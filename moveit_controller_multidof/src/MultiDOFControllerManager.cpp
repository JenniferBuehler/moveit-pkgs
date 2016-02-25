#include <moveit_controller_multidof/MultiDOFControllerManager.h>
#include <moveit_controller_multidof/ForwardingControllerHandle.h>

// XXX BEGIN MULTIDOF_CHANGE  : don't support these actions yet, just to save copying them over here too...
//  unfortunately, those headers are not included as with the catkin dependencies on moveit_simple_controller_manager
//  as of Jade, Feb '16.
// #include <moveit_simple_controller_manager/gripper_controller_handle.h>
// #include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(moveit_controller_multidof::MultiDOFControllerManager,
                       moveit_controller_manager::MoveItControllerManager);

using moveit_controller_multidof::MultiDOFControllerManager;
    
MultiDOFControllerManager::MultiDOFControllerManager() : node_handle_("~")
{

    // XXX BEGIN MULTIDOF_CHANGE
    std::string virtual_joint_name;
    std::string path_action_topic;
    std::string trajectory_action_topic;
    if (!node_handle_.hasParam("virtual_joint_name"))
    {
      ROS_INFO_STREAM_NAMED("manager","No virtual_joint_name specified, so only joint trajectories will be supported.");
    } else {
        node_handle_.getParam("virtual_joint_name",virtual_joint_name);
        if (virtual_joint_name.empty())
        {
          ROS_INFO_STREAM_NAMED("manager","Empty virtual_joint_name specified, so only joint trajectories will be supported.");
        }
    }
    if (!virtual_joint_name.empty())
    {
        if (!node_handle_.hasParam("path_navigation_action_topic"))
        {
          ROS_ERROR_STREAM_NAMED("manager","No path_navigation_action_topic specified, so only joint trajectories will be supported.");
          virtual_joint_name=""; //disable virtual joint
        }
        else 
        {
            node_handle_.getParam("path_navigation_action_topic",path_action_topic);
            ROS_INFO("Got path navigation action topic name: <%s>", path_action_topic.c_str());
            if (path_action_topic.empty())
            {
              ROS_ERROR_STREAM_NAMED("manager","Empty path_navigation_action_topic specified, so only joint trajectories will be supported.");
              virtual_joint_name=""; //disable virtual joint
            }
        }
    }
    if (!node_handle_.hasParam("joint_trajectory_action_topic"))
    {
        ROS_ERROR_STREAM_NAMED("manager","No joint_trajectory_action_topic parameter specified. Cannot use this controller.");
        return;
    }
    node_handle_.getParam("joint_trajectory_action_topic",trajectory_action_topic);
    ROS_INFO("Got joint trajectory action topic name: <%s>", trajectory_action_topic.c_str());
    if (trajectory_action_topic.empty()) {
        ROS_ERROR_STREAM_NAMED("manager","Empty joint_trajectory_action_topic specified. Cannot use this controller.");
        return;
    }
    // XXX END MULTIDOF_CHANGE

    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED("manager","No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0 ; i < controller_list.size() ; ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR_STREAM_NAMED("manager","Name and joints must be specifed for each controller");
        continue;
      }

      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        std::string action_ns;
        if (controller_list[i].hasMember("ns"))
        {
          /* TODO: this used to be called "ns", renaming to "action_ns" and will remove in the future */
          action_ns = std::string(controller_list[i]["ns"]);
          ROS_WARN_NAMED("manager","Use of 'ns' is deprecated, use 'action_ns' instead.");
        }
        else if (controller_list[i].hasMember("action_ns"))
          action_ns = std::string(controller_list[i]["action_ns"]);
        else
          ROS_WARN_NAMED("manager","Please note that 'action_ns' no longer has a default value.");

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM_NAMED("manager","The list of joints for controller " << name << " is not specified as an array");
          continue;
        }

        if (!controller_list[i].hasMember("type"))
        {
          ROS_ERROR_STREAM_NAMED("manager","No type specified for controller " << name);
          continue;
        }

        std::string type = std::string(controller_list[i]["type"]);

        ActionBasedControllerHandleBasePtr new_handle;
// XXX BEGIN MULTIDOF_CHANGE: Don't support the next two types yet
/*
        if ( type == "GripperCommand" )
        {
          new_handle.reset(new GripperControllerHandle(name, action_ns));
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            if (controller_list[i].hasMember("parallel"))
            {
              if (controller_list[i]["joints"].size() != 2)
              {
                ROS_ERROR_STREAM_NAMED("manager","Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<GripperControllerHandle*>(new_handle.get())->setParallelJawGripper(controller_list[i]["joints"][0], controller_list[i]["joints"][1]);
            }
            else
            {
              if (controller_list[i].hasMember("command_joint"))
                static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(controller_list[i]["command_joint"]);
              else
                static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(controller_list[i]["joints"][0]);
            }

            if (controller_list[i].hasMember("allow_failure"))
                static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);

            ROS_INFO_STREAM_NAMED("manager","Added GripperCommand controller for " << name );
            controllers_[name] = new_handle;
          }
        }
        else if ( type == "FollowJointTrajectory" )
        {
          new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
          if (static_cast<FollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM_NAMED("manager","Added FollowJointTrajectory controller for " << name );
            controllers_[name] = new_handle;
          }
        }
        else
*/
// XXX END MULTIDOF_CHANGE

// XXX BEGIN MULTIDOF_CHANGE : Support the "FollowRobotTrajectory" type.
        if ( type == "FollowRobotTrajectory" )
        {
          ROS_INFO_STREAM_NAMED("mangager","Adding 'FollowRobotTrajectory'...");
          new_handle.reset(new ForwardingControllerHandle(trajectory_action_topic, path_action_topic, virtual_joint_name));
          ROS_INFO_STREAM_NAMED("manager","Added FollowRobotTrajectory controller for " << name );
          controllers_[name] = new_handle;
        }
// XXX END MULTIDOF_CHANGE
        else
        {
          ROS_ERROR_STREAM_NAMED("manager","Unknown controller type: " << type.c_str());
          continue;
        }
        if (!controllers_[name])
        {
          controllers_.erase(name);
          continue;
        }

        /* add list of joints, used by controller manager and moveit */
        for (int j = 0 ; j < controller_list[i]["joints"].size() ; ++j)
          controllers_[name]->addJoint(std::string(controller_list[i]["joints"][j]));
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED("manager","Unable to parse controller information");
      }
    }
}

moveit_controller_manager::MoveItControllerHandlePtr MultiDOFControllerManager::getControllerHandle(const std::string &name)
{
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      ROS_FATAL_STREAM_NAMED("manager","No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
}

void MultiDOFControllerManager::getControllersList(std::vector<std::string> &names)
{
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin() ; it != controllers_.end() ; ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM_NAMED("manager","Returned " << names.size() << " controllers in list");
}

void MultiDOFControllerManager::getControllerJoints(const std::string &name, std::vector<std::string> &joints)
{
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN_NAMED("manager","The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
}


moveit_controller_manager::MoveItControllerManager::ControllerState MultiDOFControllerManager::getControllerState(const std::string &name)
{
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
}

