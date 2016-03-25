#ifndef MOVEIT_OBJECTS_OBJECTMESSAGEGENERATOR_H
#define MOVEIT_OBJECTS_OBJECTMESSAGEGENERATOR_H

#include <ros/ros.h>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/thread/mutex.hpp>

#include <moveit_object_handling/MoveItCollisionMatrixManipulator.h>

namespace moveit_object_handling
{

/**
 * Takes objects of type object_msgs/Object published a topic and transforms and re-publishes it
 * into a moveit_msgs/CollisionObject.
 *
 * The object information is read from incoming object_msgs/Object messages. If not all information
 * is available, a service of type object_msgs/ObjectInfoRequest.srv is sent, e.g. to get the
 * geometry details when required.
 *
 * There are currently two ways to send object information to MoveIt!:
 *
 * 1. Publishing the collision object on /collision_objects which is read by move_group.
 * 2. Another approach is described in
 *      [this PR2 tutorial](http://docs.ros.org/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html),
 *      in section *"Add object to environment": publising a planning scene diff (message moveit_msgs::PlanningScene)*.
 *
 * The approach to be taken can be set with a ROS parameter is set for it.
 *
 * The parameters for this class can be specified in a YAML file,
 * which needs to be loaded onto the ROS parameter server
 * under **namespace moveit_object_handling**. An example is given in the directory *config*, filename "CollisionObjectsGenerator.yaml"
 *
 * \author Jennifer Buehler
 **/
class ObjectMessageGenerator
{
private:

    typedef object_msgs::Object ObjectMsg;
    typedef object_msgs::ObjectInfo ObjectInfoMsg;
    typedef std::map<std::string, moveit_msgs::CollisionObject> ObjToPublishMap;

public:

    ObjectMessageGenerator(ros::NodeHandle& _node_priv, ros::NodeHandle& _node);
    virtual ~ObjectMessageGenerator();

    bool isConnected() const;

private:


    void connectPub(const ros::SingleSubscriberPublisher& p);

    void disconnectPub(const ros::SingleSubscriberPublisher& p);

    void publishCollisionsEvent(const ros::TimerEvent& e);

    /**
      * Callback for object message
     */
    void receiveObject(const ObjectMsg& msg);

    moveit_msgs::CollisionObject getCollisionGeometry(const std::string& name);

    /**
     * updates pose data in obj with information in newObj, leaves all other fields untouched
     */
    void updatePose(const ObjectMsg& newObj, moveit_msgs::CollisionObject& obj);

    /**
     * Helper to transfer contents from ObjectMsg to moveit_msgs::CollisionObject
     */
    moveit_msgs::CollisionObject transferContent(const ObjectMsg& msg, bool skipGeometry);

    std::vector<moveit_msgs::CollisionObject> getCurrentCollisionObjects(bool only_names = true);
    std::set<std::string> getCurrentCollisionObjectNames();

    std::vector<moveit_msgs::AttachedCollisionObject> getCurrentAttachedCollisionObjects();
    std::set<std::string> getCurrentAttachedCollisionObjectNames();

    ros::NodeHandle node_priv, node;
    std::string OBJECTS_TOPIC;
    std::string REQUEST_OBJECTS_TOPIC;
    std::string COLLISION_OBJECT_TOPIC;
    std::string GET_PLANNING_SCENE_SERVICE;
    std::string SET_PLANNING_SCENE_TOPIC;
    float PUBLISH_COLLISION_RATE;
    bool USE_PLANNING_SCENE_DIFF;

    ros::Publisher collision_pub;
    ros::Publisher planning_scene_pub;

    ros::Subscriber object_sub;

    ros::ServiceClient object_info_client;

    ros::ServiceClient planning_scene_client;


    // all objects which were already added
    std::set<std::string> addedObjects;

    // objects to skip as collision objects:
    // whenever an Object.msg message arrives to
    // add this object name, skip it.
    std::set<std::string> skipObjects;

    // links of the robot to always be allowed to
    // collide with *any* new collision object arriving.
    // Technically can be used for any objects (not only
    // robot links), but it's intended to disable collisions
    // with parts of the robot only.
    std::vector<std::string> allowedCollisionLinks;

    MoveItCollisionMatrixManipulator acmManip;

    ObjToPublishMap objsToPublish;

    boost::mutex mutex; //mutex for addedObjects and objsToPublish

    ros::Timer publishCollisionsTimer;

    //is set to false until addedObjects is initialized
    bool initExistingObj;
};

}  // namespace moveit_object_handling

#endif  // MOVEIT_OBJECTS_OBJECTMESSAGEGENERATOR_H
