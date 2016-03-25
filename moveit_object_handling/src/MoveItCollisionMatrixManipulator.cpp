#include <moveit_object_handling/MoveItCollisionMatrixManipulator.h>
#include <moveit_object_handling/MoveItHelpers.h>

#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <Eigen/Core>

#define DEFAULT_SET_PLANNING_SCENE_TOPIC "/planning_scene"
#define DEFAULT_GET_PLANNING_SCENE_TOPIC "/get_planning_scene"

using moveit_object_handling::MoveItCollisionMatrixManipulator;

MoveItCollisionMatrixManipulator::MoveItCollisionMatrixManipulator(ros::NodeHandle& nh)
{
    ros::NodeHandle _node("/moveit_object_handling");

    GET_PLANNING_SCENE_TOPIC = DEFAULT_GET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_get_planning_scene_topic", GET_PLANNING_SCENE_TOPIC, GET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_get_planning_scene_topic: <%s>", GET_PLANNING_SCENE_TOPIC.c_str());


    SET_PLANNING_SCENE_TOPIC = DEFAULT_SET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_set_planning_scene_topic", SET_PLANNING_SCENE_TOPIC, SET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_set_planning_scene_topic: <%s>", SET_PLANNING_SCENE_TOPIC.c_str());

    planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(SET_PLANNING_SCENE_TOPIC, 1);
    planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_TOPIC);

    // only done to ensure connections are made with the planning scene
    // topic/service, this works better together with MoveIt! launch files
    ros::Duration(0.2).sleep();
}

MoveItCollisionMatrixManipulator::~MoveItCollisionMatrixManipulator()
{}


void MoveItCollisionMatrixManipulator::expandMoveItCollisionMatrix(const std::string& name,
        moveit_msgs::AllowedCollisionMatrix& m, bool default_val)
{

    for (int i = 0; i < m.entry_names.size(); ++i)
    {
        m.entry_values[i].enabled.push_back(default_val);
    }

    m.entry_names.push_back(name);

    moveit_msgs::AllowedCollisionEntry e;
    e.enabled.assign(m.entry_names.size(), default_val);
    m.entry_values.push_back(e);
}

bool MoveItCollisionMatrixManipulator::getCurrentMoveItAllowedCollisionMatrix(
    moveit_msgs::AllowedCollisionMatrix& matrix)
{

    moveit_msgs::GetPlanningScene srv;

    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene");
        return false;
    }

    matrix = srv.response.scene.allowed_collision_matrix;
    if (matrix.entry_names.empty())
    {
        ROS_ERROR("Collision matrix should not be empty");
        return false;
    }

    //ROS_INFO_STREAM("Matrix: "<<matrix);
    return true;
}

bool MoveItCollisionMatrixManipulator::setAllowedMoveItCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& m)
{
    moveit_msgs::PlanningScene planning_scene;

    if (planning_scene_publisher.getNumSubscribers() < 1)
    {
        ROS_ERROR("Setting collision matrix won't have any effect!");
        return false;
    }
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = m;
    planning_scene_publisher.publish(planning_scene);
    return true;
}


bool MoveItCollisionMatrixManipulator::addAllowedMoveItCollision(const std::string& name,
        const std::vector<std::string>& linkNames)
{
    moveit_msgs::AllowedCollisionMatrix m;
    if (!getCurrentMoveItAllowedCollisionMatrix(m))
    {
        return false;
    }

    //ROS_INFO_STREAM("Allowed collisoin: "<<m);

    std::vector<std::string>::iterator objEntry = ensureExistsInACM(name, m, false);
    int obj_idx = objEntry - m.entry_names.begin();

    std::vector<std::string>::const_iterator it;
    for (it = linkNames.begin(); it != linkNames.end(); ++it)
    {
        std::vector<std::string>::iterator linkEntry = ensureExistsInACM(*it, m, false);
        int link_idx = linkEntry - m.entry_names.begin();
        m.entry_values[link_idx].enabled[obj_idx] = true;
        m.entry_values[obj_idx].enabled[link_idx] = true;
    }

    setAllowedMoveItCollisionMatrix(m);
    return true;
}



std::vector<std::string>::iterator MoveItCollisionMatrixManipulator::ensureExistsInACM(
    const std::string& name, moveit_msgs::AllowedCollisionMatrix& m, bool initFlag)
{
    std::vector<std::string>::iterator name_entry = std::find(m.entry_names.begin(), m.entry_names.end(), name);
    if (name_entry == m.entry_names.end())
    {
        ROS_DEBUG_STREAM("Could not find object " << name
                         << " in collision matrix. Inserting.");
        expandMoveItCollisionMatrix(name, m, initFlag);
        // re-assign the 'name_entry' iterator to the new entry_names place
        name_entry = std::find(m.entry_names.begin(), m.entry_names.end(), name);
        if (name_entry == m.entry_names.end())
        {
            ROS_ERROR("consistency, name should now be in map");
        }
    }
    return name_entry;
}


bool MoveItCollisionMatrixManipulator::setAllowedMoveItCollision(const std::string& name1, const std::string& name2, const bool flag)
{

    moveit_msgs::AllowedCollisionMatrix m;
    if (!getCurrentMoveItAllowedCollisionMatrix(m))
    {
        return false;
    }

    //ROS_INFO_STREAM("Allowed collisoin: "<<m);

    std::vector<std::string>::iterator name1_entry = ensureExistsInACM(name1, m, false);
    std::vector<std::string>::iterator name2_entry = ensureExistsInACM(name2, m, false);

    int name1_entry_idx = name1_entry - m.entry_names.begin();
    int name2_entry_idx = name2_entry - m.entry_names.begin();

    m.entry_values[name2_entry_idx].enabled[name1_entry_idx] = flag;
    m.entry_values[name1_entry_idx].enabled[name2_entry_idx] = flag;

    setAllowedMoveItCollisionMatrix(m);
    return true;
}





bool MoveItCollisionMatrixManipulator::attachMoveItObjectToRobot(const std::string& name,
        const std::string& link_name, const std::vector<std::string>& allowedTouchLinks)
{

    if (planning_scene_publisher.getNumSubscribers() < 1)
    {
        ROS_ERROR("attachObjectToRobot: No node subscribed to planning scene publisher, can't refresh octomap. %s",
                  SET_PLANNING_SCENE_TOPIC.c_str());
        return false;
    }

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    ROS_INFO("Requesting scene to modify object attachment..");

    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene");
        return false;
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    ROS_INFO("Now attaching object to robot");

    moveit_msgs::CollisionObject collObj;
    if (!hasObject(name, srv.response.scene.world.collision_objects, collObj))
    {
        ROS_ERROR("Object %s was not in the scene", name.c_str());
        return false;
    }
    else
    {
        //remove object from planning scene
        collObj.operation = collObj.REMOVE;
        planning_scene.world.collision_objects.push_back(collObj);
    }

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.header.frame_id = link_name;
    attached_object.object = collObj;
    attached_object.touch_links = allowedTouchLinks;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    attached_object.object.operation = attached_object.object.ADD;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);


    planning_scene_publisher.publish(planning_scene);

    ROS_INFO("Have attached object. %s", __FILE__);

    while (true)
    {
        srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        ROS_INFO("Requesting scene to see if object is attached..");

        if (!planning_scene_client.call(srv))
        {
            ROS_ERROR("Can't obtain planning scene");
            return false;
        }

        ROS_INFO("Scene obtained");
        moveit_msgs::AttachedCollisionObject o;
        if (hasObject(name, srv.response.scene.robot_state.attached_collision_objects, o))
        {
            ROS_INFO("Scene is updated with attached object.");
            break;
        }
        ROS_INFO("Waiting for scene update to attach object...");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Successfully attached object.");

    return true;
}


bool MoveItCollisionMatrixManipulator::detachMoveItObjectFromRobot(const std::string& name)
{

    if (planning_scene_publisher.getNumSubscribers() < 1)
    {
        ROS_ERROR("attachObjectToRobot: No node subscribed to planning scene publisher, can't refresh octomap. %s",
                  SET_PLANNING_SCENE_TOPIC.c_str());
        return false;
    }

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

    ROS_INFO("Requesting scene to modify object attachment..");

    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene");
        return false;
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    ROS_INFO("Now detaching object from robot");

    moveit_msgs::AttachedCollisionObject attObj;
    if (!hasObject(name, srv.response.scene.robot_state.attached_collision_objects, attObj))
    {
        ROS_ERROR("Object %s was not attached to robot", name.c_str());
        return false;
    }
    else
    {
        //remove object from planning scene
        attObj.object.operation = attObj.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.push_back(attObj);
    }

    moveit_msgs::CollisionObject collision_object = attObj.object;
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);

    planning_scene_publisher.publish(planning_scene);

    ROS_INFO("Have deattached object. %s", __FILE__);

    while (true)
    {
        srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

        ROS_INFO("Requesting scene to see if object is detached..");

        if (!planning_scene_client.call(srv))
        {
            ROS_ERROR("Can't obtain planning scene");
            return false;
        }

        ROS_INFO("Scene obtained");
        moveit_msgs::CollisionObject o;
        if (hasObject(name, srv.response.scene.world.collision_objects, o))
        {
            ROS_INFO("Scene is updated with detached object.");
            break;
        }
        ROS_INFO("Waiting for scene update to detach object...");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Successfully detached object.");

    return true;
}


bool MoveItCollisionMatrixManipulator::getMoveItScene(octomap_msgs::OctomapWithPose & octomap,
        std::vector<moveit_msgs::CollisionObject>& collision_objects)
{

    moveit_msgs::GetPlanningScene srv;

    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene");
        return false;
    }

    moveit_msgs::PlanningScene& planning_scene = srv.response.scene;

    octomap = planning_scene.world.octomap;
    collision_objects = planning_scene.world.collision_objects;
    return true;
}

bool MoveItCollisionMatrixManipulator::hasObject(const std::string& name,
        const std::vector<moveit_msgs::AttachedCollisionObject>& objs, moveit_msgs::AttachedCollisionObject& o)
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


bool MoveItCollisionMatrixManipulator::hasObject(const std::string& name,
        const std::vector<moveit_msgs::CollisionObject>& objs, moveit_msgs::CollisionObject& o)
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


bool MoveItCollisionMatrixManipulator::removeObject(const std::string& name,
        std::vector<moveit_msgs::CollisionObject>& objs)
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


bool MoveItCollisionMatrixManipulator::removeObject(const std::string& name,
        std::vector<moveit_msgs::AttachedCollisionObject>& objs)
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


