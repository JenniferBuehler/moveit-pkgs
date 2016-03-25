#ifndef OBJECT_MOVEIT_GRASPEDOBJECTHANDLER_H
#define OBJECT_MOVEIT_GRASPEDOBJECTHANDLER_H

// #include <capabilities/ObjectInfoManager.h>
// #include <convenience_ros_functions/ROSFunctions.h>
#include <ros/ros.h>
#include <string>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace moveit_object_handling
{

/**
 * Abstract class to serve as a helper for handling attaching and detaching of objects to a robot.
 *
 * TODO: At some stage, this will move to a more general package than this one in MoveIt!. This will
 * happen as soon as another implementation of the Handler is needed, and a general package (maybe
 * in grasp-pkgs repository) is created.
 *
 * \author Jennifer Buehler
 * \date February 2016
 */
class GraspedObjectHandler
{
public:
    GraspedObjectHandler()
    {
    }
    virtual ~GraspedObjectHandler()
    {
    }

    virtual bool attachObjectToRobot(const std::string& object_name, const std::string& attach_link_name) = 0;
    /**
     */
    virtual bool detachObjectFromRobot(const std::string& object_name) = 0;

};


/**
 * Implementation of GraspedObjectHandler for MoveIt! collision object.
 * \author Jennifer Buehler
 * \date February 2016
 */
class GraspedObjectHandlerMoveIt: public GraspedObjectHandler
{
public:
    /**
     * \param _gripperLinkNames names of the gripper links. Those are the links which are allowed to collide
     *          with the object being grasped (moveit_msgs::AttachedCollisionObject.touch_links)
     */
    GraspedObjectHandlerMoveIt(
        ros::NodeHandle& n, const std::vector<std::string>& _gripperLinkNames,
        const std::string& get_planning_scene_service = "/get_planning_scene",
        const std::string& set_planning_scene_topic = "/planning_scene");

    virtual bool attachObjectToRobot(const std::string& object_name, const std::string& attach_link_name);

    virtual bool detachObjectFromRobot(const std::string& object_name);

    void waitForSubscribers();

private:
    bool attachObjectToRobot(const std::string& name, const std::string& link_name, const std::vector<std::string>& allowedTouchLinks);

    static bool hasObject(const std::string& name, const std::vector<moveit_msgs::AttachedCollisionObject>& objs, moveit_msgs::AttachedCollisionObject& o);

    static bool hasObject(const std::string& name, const std::vector<moveit_msgs::CollisionObject>& objs, moveit_msgs::CollisionObject& o);

    static bool removeObject(const std::string& name, std::vector<moveit_msgs::CollisionObject>& objs);

    static bool removeObject(const std::string& name, std::vector<moveit_msgs::AttachedCollisionObject>& objs);

    void subscribeAndAdvertise(const std::string& get_planning_scene_service = "/get_planning_scene",
                               const std::string& set_planning_scene_topic = "/planning_scene");

    bool transformPose(const geometry_msgs::Pose& pose, const std::string& from_frame, const std::string& to_frame, geometry_msgs::Pose& p);

    /**
     * Transform all object poses to the given frame
     */
    bool transformCollisionObject(const std::string& to_frame, moveit_msgs::CollisionObject& collision_object);

    ros::ServiceClient moveit_planning_scene_client;
    ros::Publisher moveit_planning_scene_publisher;

    ros::NodeHandle& node;

    std::vector<std::string> gripperLinkNames;
};

}

#endif  // OBJECT_MOVEIT_GRASPEDOBJECTHANDLER_H
