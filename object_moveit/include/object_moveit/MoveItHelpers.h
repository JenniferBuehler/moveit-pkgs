#ifndef OBJECT_MOVEIT_MOVEITHELPERS
#define OBJECT_MOVEIT_MOVEITHELPERS

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit_msgs/Constraints.h>
#include <eigen_conversions/eigen_msg.h>

namespace object_moveit
{

class MoveItHelpers
{

public:
    typedef boost::shared_ptr<moveit_msgs::PositionConstraint> PositionConstraintPtr;
    typedef boost::shared_ptr<shape_msgs::SolidPrimitive> SolidPrimitivePtr;

    /**
     */
    MoveItHelpers() {}

    ~MoveItHelpers() {}

    /**
     * Adds goal constraints for the link to be at this pose.
     * \param type if 0, only position is considered. If 1, position and
     *      orientation are considered, and if 2 then only orientation is considered.
     */
    static moveit_msgs::Constraints getPoseConstraint(const std::string &link_name,
            const geometry_msgs::PoseStamped &pose, double tolerance_pos, double tolerance_angle, int type);

    static moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string& link_name,
            const geometry_msgs::QuaternionStamped& quat,
            const float& x_tolerance,
            const float& y_tolerance,
            const float& z_tolerance);

    /**
     * Gets the joint constraint such that it corresponds to the passed joint_state
     */
    static moveit_msgs::Constraints getJointConstraint(const std::string& link_name,
            const sensor_msgs::JointState& joint_state, const float& joint_tolerance);


    /**
     * \param maxArmReachDistance the maximum distance the arm can reach (radius of the reaching sphere).
     * Can be an overestimation of it, but should not underestimate.
     */
    /*static PositionConstraintPtr getCombinedPoseConstraint(
            const std::string &link_name,
            const geometry_msgs::PoseStamped &target_pose,
            float tolerance_pos,
            const geometry_msgs::PoseStamped& target_pose2,
            float tolerance_pos2,
            float maxArmReachDistance);
            */

    /**
     * \param maxArmReachDistance the maximum distance the arm can reach (radius of the reaching sphere).
     * Can be an overestimation of it, but should not underestimate.
     */
    static PositionConstraintPtr getSpherePoseConstraint(
        const std::string &link_name,
        const geometry_msgs::PoseStamped &target_pose,
        float maxArmReachDistance);

    /**
     * \param boxOrgin the origin of the box
     */
    static moveit_msgs::PositionConstraint getBoxConstraint(const std::string &link_name,
            const geometry_msgs::PoseStamped& boxOrigin, const double& x, const double& y, const double& z);

    /* static bool getCylinderConstraint(const std::string &link_name, const geometry_msgs::PoseStamped& from,
            const geometry_msgs::PoseStamped& to, const double& radius, moveit_msgs::PositionConstraint& result);
     */

    /**
     * Like getCylinderConstraint() with two input stamped poses, but here 'dir' is specified
     * relative to from, it is a moving direction, not a position.
     */
    /*static moveit_msgs::PositionConstraint getCylinderConstraint(const std::string &link_name,
            const geometry_msgs::PoseStamped& from, const geometry_msgs::Vector3& dir, const double& radius); */


    /**
     * returns a constraint with a cone which at <from> has radius <radius_start>,
     * and goes along <dir>, where it has radius <radius_end>.
     */
    /*static moveit_msgs::PositionConstraint getConeConstraint(const std::string &link_name,
            const geometry_msgs::PoseStamped& from, const geometry_msgs::Vector3& dir,
            const double& radius_start, const double& radius_end);*/


    /**
     */
    static SolidPrimitivePtr getConeBV(const Eigen::Vector3d& _fromPose,
                                       const Eigen::Quaterniond& _fromOrientation, const Eigen::Vector3d& _direction,
                                       const double& radius_start, const double& radius_end,
                                       Eigen::Vector3d& bv_pose, Eigen::Quaterniond& bv_orientation);

    /**
     */
    static SolidPrimitivePtr getCylinderBV(const Eigen::Vector3d& _fromPose,
                                           const Eigen::Quaterniond& _fromOrientation,
                                           const Eigen::Vector3d& _direction, const double& radius,
                                           Eigen::Vector3d& bv_pose, Eigen::Quaterniond& bv_orientation);
};

}  // namespace object_moveit

#endif  // OBJECT_MOVEIT_MOVEITHELPERS
