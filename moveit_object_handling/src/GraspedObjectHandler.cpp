#include <moveit_object_handling/GraspedObjectHandler.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <convenience_ros_functions/ROSFunctions.h>

using moveit_object_handling::GraspedObjectHandler;
using moveit_object_handling::GraspedObjectHandlerMoveIt;

GraspedObjectHandlerMoveIt::GraspedObjectHandlerMoveIt(
    ros::NodeHandle& n, const std::vector<std::string>& _gripperLinkNames,
    const std::string& get_planning_scene_service,
    const std::string& set_planning_scene_topic):
    node(n),
    gripperLinkNames(_gripperLinkNames)
{
    // global_frame(_global_frame),
    // object_info_manager(_obj_info_manager)   {

    convenience_ros_functions::ROSFunctions::initSingleton();
    subscribeAndAdvertise(get_planning_scene_service, set_planning_scene_topic);
}

bool GraspedObjectHandlerMoveIt::attachObjectToRobot(const std::string& object_name, const std::string& attach_link_name) {

    if (!attachObjectToRobot(object_name, attach_link_name, gripperLinkNames)) {
        ROS_ERROR("Could not attach object to robot");
        return false;
    }

    ROS_INFO_STREAM("Have attached object "<<object_name<<" to "<<attach_link_name);
    return true;
}

bool GraspedObjectHandlerMoveIt::detachObjectFromRobot(const std::string& object_name)
{

    if (moveit_planning_scene_publisher.getNumSubscribers() < 1)
    {
        ROS_WARN("detachObjectToRobot: No node subscribed to planning scene publisher.");
        return false;
    }

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

    //ROS_INFO("Requesting scene to modify object attachment..");

    if (!moveit_planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene");
        return false;
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    ROS_INFO("Now detaching object '%s' from robot", object_name.c_str());

    moveit_msgs::AttachedCollisionObject attObj;
    if (!hasObject(object_name, srv.response.scene.robot_state.attached_collision_objects, attObj))
    {
        ROS_WARN("GraspedObjectHandlerMoveIt: Object %s was not attached to robot, but it was tried to detach it.", object_name.c_str());
        return true;
    }

    //remove object from planning scene
    attObj.object.operation = attObj.object.REMOVE;
    planning_scene.robot_state.attached_collision_objects.push_back(attObj);
    planning_scene.robot_state.is_diff = true;

    // Add object to the planning scene again
    // INFO: This was taken out, as the object recognition (along with MoveI! collision
    // object generator) should take care of adding the object to the scene again.
    // Transform the object into the link coordinate frame:
    /*moveit_msgs::CollisionObject collision_object=attObj.object;
    if (!transformCollisionObject(target_frame, collision_object)) {
        ROS_ERROR("GraspedObjectHandler: Could nto transform object to world frame");
        return false;
    }
    //send object as MoveIt collision object:
    moveit_msgs::CollisionObject collObj=attObj.object;
    collObj.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene.world.collision_objects.push_back(collObj);*/

    moveit_planning_scene_publisher.publish(planning_scene);

    //ROS_INFO("Successfully detached object.");
    return true;
}


bool GraspedObjectHandlerMoveIt::attachObjectToRobot(const std::string& name,
        const std::string& link_name, const std::vector<std::string>& allowedTouchLinks)
{

    ROS_INFO("GraspedObjectHandlerMoveIt: Attaching %s to %s", name.c_str(), link_name.c_str());

    if (moveit_planning_scene_publisher.getNumSubscribers() < 1)
    {
        ROS_ERROR("GraspedObjectHandlerMoveIt: attachObjectToRobot: No node subscribed to planning scene publisher.");
        return false;
    }

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    //ROS_INFO("Requesting scene to modify object attachment..");

    if (!moveit_planning_scene_client.call(srv))
    {
        ROS_ERROR("GraspedObjectHandlerMoveIt: Can't obtain planning scene in order to attach object.");
        return false;
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    //ROS_INFO("Now attaching object to robot");

    moveit_msgs::CollisionObject collObj;
    if (!hasObject(name, srv.response.scene.world.collision_objects, collObj))
    {
        ROS_ERROR("GraspedObjectHandlerMoveIt: Object %s was not in the scene, but it was tried to attach it to robot.", name.c_str());
        return false;
    }
    else
    {
        // remove object from planning scene because now it's attached to the robot.
        collObj.operation = collObj.REMOVE;
        planning_scene.world.collision_objects.push_back(collObj);
    }

    //transform the object into the link coordinate frame
    if (!transformCollisionObject(link_name, collObj))
    {
        ROS_ERROR("GraspedObjectHandlerMoveIt: Could not transform object to link frame");
        return false;
    }

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object = collObj;
    attached_object.object.header.frame_id = link_name;
    attached_object.link_name = link_name;
    attached_object.touch_links = allowedTouchLinks;

    attached_object.object.operation = attached_object.object.ADD;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;

    moveit_planning_scene_publisher.publish(planning_scene);

    //ROS_INFO("Have attached object. %s",__FILE__);

    while (true)
    {
        srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        //ROS_INFO("Requesting scene to see if object is attached..");

        if (!moveit_planning_scene_client.call(srv))
        {
            ROS_ERROR("GraspedObjectHandlerMoveIt: Can't obtain planning scene");
            return false;
        }

        //ROS_INFO("Scene obtained");
        moveit_msgs::AttachedCollisionObject o;
        if (hasObject(name, srv.response.scene.robot_state.attached_collision_objects, o))
        {
            ROS_INFO("GraspedObjectHandlerMoveIt: Scene is updated with attached object.");
            break;
        }
        ROS_INFO("GraspedObjectHandlerMoveIt: Waiting for scene update to attach object...");
        ros::Duration(0.5).sleep();
    }

    //ROS_INFO("Successfully attached object.");

    return true;
}




bool GraspedObjectHandlerMoveIt::hasObject(const std::string& name, const std::vector<moveit_msgs::AttachedCollisionObject>& objs, moveit_msgs::AttachedCollisionObject& o)
{
    for (int i = 0; i < objs.size(); ++i)
    {
        if (objs[i].object.id == name)
        {
            o = objs[i];
            return true;
        }
    }
    return false;
}


bool GraspedObjectHandlerMoveIt::hasObject(const std::string& name, const std::vector<moveit_msgs::CollisionObject>& objs, moveit_msgs::CollisionObject& o)
{
    for (int i = 0; i < objs.size(); ++i)
    {
        if (objs[i].id == name)
        {
            o = objs[i];
            return true;
        }
    }
    return false;
}


bool GraspedObjectHandlerMoveIt::removeObject(const std::string& name, std::vector<moveit_msgs::CollisionObject>& objs)
{
    for (int i = 0; i < objs.size(); ++i)
    {
        if (objs[i].id == name)
        {
            objs.erase(objs.begin() + i);
            return true;
        }
    }
    return false;
}


bool GraspedObjectHandlerMoveIt::removeObject(const std::string& name, std::vector<moveit_msgs::AttachedCollisionObject>& objs)
{
    for (int i = 0; i < objs.size(); ++i)
    {
        if (objs[i].object.id == name)
        {
            objs.erase(objs.begin() + i);
            return true;
        }
    }
    return false;
}


void GraspedObjectHandlerMoveIt::subscribeAndAdvertise(const std::string& get_planning_scene_service,
        const std::string& set_planning_scene_topic)
{
    moveit_planning_scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>(get_planning_scene_service);
    moveit_planning_scene_publisher = node.advertise<moveit_msgs::PlanningScene>(set_planning_scene_topic, 1);
}

void GraspedObjectHandlerMoveIt::waitForSubscribers()
{
    while (moveit_planning_scene_publisher.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for subscribers...");
        ros::Duration(0.5).sleep();
    }
}

bool GraspedObjectHandlerMoveIt::transformPose(const geometry_msgs::Pose& pose,
        const std::string& from_frame, const std::string& to_frame, geometry_msgs::Pose& p)
{
    if (to_frame.empty() || from_frame.empty())
    {
        ROS_ERROR("GraspObjectHandler::transformPose(): Both frames must be set.");
        return false;
    }

    if (from_frame == to_frame)
    {
        p = pose;
        return true;
    }

    geometry_msgs::PoseStamped newPose, resultPose;
    newPose.pose = pose;
    newPose.header.frame_id = from_frame;
    newPose.header.stamp = ros::Time(0); //in order to get the most recent transform
    if (convenience_ros_functions::ROSFunctions::Singleton()->transformPose(newPose, to_frame, resultPose, 1) != 0)
    {
        ROS_ERROR("ObjectInfoManager: Transform into frame %s failed. Ignoring transform.", to_frame.c_str());
        return false;
    }
    p = resultPose.pose;
    return true;
}

/**
 * Transform all object poses to the given frame
 */
bool GraspedObjectHandlerMoveIt::transformCollisionObject(const std::string& to_frame,
        moveit_msgs::CollisionObject& collision_object)
{

    for (int i = 0; i < collision_object.primitive_poses.size(); ++i)
    {
        geometry_msgs::Pose& p = collision_object.primitive_poses[i];
        if (!transformPose(p, collision_object.header.frame_id, to_frame, p))
        {
            ROS_ERROR("GraspObjectHandler: Could not transform object to link frame %s", to_frame.c_str());
            return false;
        }
        if (i < collision_object.mesh_poses.size())
        {
            p = collision_object.mesh_poses[i];
            collision_object.mesh_poses[i] = p;
            if (!transformPose(p, collision_object.header.frame_id, to_frame, p))
            {
                ROS_ERROR("GraspObjectHandler: Could not transform object mesh to link frame %s", to_frame.c_str());
                return false;
            }
        }
        if (i < collision_object.plane_poses.size())
        {
            p = collision_object.plane_poses[i];
            collision_object.plane_poses[i] = p;
            if (!transformPose(p, collision_object.header.frame_id, to_frame, p))
            {
                ROS_ERROR("GraspObjectHandler: Could not transform object plane to link frame %s", to_frame.c_str());
                return false;
            }
        }
    }
    collision_object.header.frame_id = to_frame;
    collision_object.header.stamp = ros::Time::now();
    return true;
}
