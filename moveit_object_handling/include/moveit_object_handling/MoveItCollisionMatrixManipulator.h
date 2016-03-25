#ifndef OBJECT_MOVEIT_MOVEITCOLLISIONMATRIXMANIPULATOR
#define OBJECT_MOVEIT_MOVEITCOLLISIONMATRIXMANIPULATOR

/*#include <capabilities/MoveItHelpers.h>
#include <phd_experiments/general_helpers.h>
*/
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/CollisionObject.h>

/*#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetCartesianPath.h>
*/

#include <ros/ros.h>

namespace moveit_object_handling
{

/**
 * \brief Provides helper functions to manipulate the MoveIt! Allowed Collision Matrix (ACM).
 *
 * The ACM saves the expensive collision checking operation by a matrix of flags, where
 * when the flag is 1, it specifies that a collision check between the two objects is not needed
 * (either the two never collide, or have explicitly been allowed to collide).
 * The collision matrix is a symmetric matrix.
 *
 * The collision matrix is retrieved and set by the ROS MoveIt! planning scene service and topic.
 *
 * \author Jennifer Buehler
 * \date February 2016
 */
class MoveItCollisionMatrixManipulator
{
public:
    MoveItCollisionMatrixManipulator(ros::NodeHandle& nh);
    ~MoveItCollisionMatrixManipulator();

    /**
     * Gets the current collision matrix of MoveIt!.
     */
    bool getCurrentMoveItAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& matrix);

    /**
     * Sets the given collision matrix in MoveIt!.
     */
    bool setAllowedMoveItCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& m);

    /**
     * Allows collisions between collision object \e name and all links of the robot
     * given in \e linkNames
     */
    bool addAllowedMoveItCollision(const std::string& name, const std::vector<std::string>& linkNames);

    /**
     * Set the pair <name1, name2> in the collision matrix with the \e flag. "true" is for allowed
     * collisions (no collision checks needed).
     * If entries for name1 and name2 don't exist, they are added to the collision matrix,
     * and by default are not allowed to collide with any of the other entries (set to "false" for
     * all other objects in ACM).
     */
    bool setAllowedMoveItCollision(const std::string& name1, const std::string& name2, const bool flag);

    /**
     * Attaches the object which is known to MoveIt! by \e name to the link \e link_name
     * of the robot. The links of the hand which are allowed to touch the object are to
     * be given in \e allowedTouchLinks.
     * \return false if MoveIt! planning scene couldn't be reached or if the object is
     * not known as collision object to MoveIt!.
     */
    bool attachMoveItObjectToRobot(const std::string& name, const std::string& link_name,
                                   const std::vector<std::string>& allowedTouchLinks);

    /**
     * Detaches the object which has previously been attached with attachMoveItObjectToRobot()
     * from the robot. Returns false if it wasn't previously attached or MoveIt! planning scene
     * could not be reached.
     */
    bool detachMoveItObjectFromRobot(const std::string& name);

private:

    /**
     * Expands the collision matrix by appending one row, and one column, and set all fields to \e default_val.
     */
    void expandMoveItCollisionMatrix(const std::string& name, moveit_msgs::AllowedCollisionMatrix& m, bool default_val);


    bool getMoveItScene(octomap_msgs::OctomapWithPose & octomap,
                        std::vector<moveit_msgs::CollisionObject>& collision_objects);

    bool hasObject(const std::string& name, const std::vector<moveit_msgs::AttachedCollisionObject>& objs,
                   moveit_msgs::AttachedCollisionObject& o);

    bool hasObject(const std::string& name, const std::vector<moveit_msgs::CollisionObject>& objs,
                   moveit_msgs::CollisionObject& o);

    bool removeObject(const std::string& name, std::vector<moveit_msgs::CollisionObject>& objs);

    bool removeObject(const std::string& name, std::vector<moveit_msgs::AttachedCollisionObject>& objs);

    /**
     * Makes sure an entry for this name is added to the collision matrix if it doesn't exist,
     * and then returns the iterator to the ACM.entry_names for it.
     */
    std::vector<std::string>::iterator ensureExistsInACM(
        const std::string& name, moveit_msgs::AllowedCollisionMatrix& m, bool initFlag);


    std::string SET_PLANNING_SCENE_TOPIC;
    std::string GET_PLANNING_SCENE_TOPIC;

    ros::Publisher planning_scene_publisher;
    ros::ServiceClient planning_scene_client;
};  // class

}  // namespace
#endif
