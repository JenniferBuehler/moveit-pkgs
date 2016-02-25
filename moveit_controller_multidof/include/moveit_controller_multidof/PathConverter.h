#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>


namespace moveit_controller_multidof  {

/**
 * \brief Provides methods to transform nav_msgs::Path to geometry_msgs/Transform[] and vice versa
 *
 * \author Jennifer Buehler
 */
class TransformPathConverter {
private:
    typedef Eigen::Transform<double,3,Eigen::Affine> EigenTransform;
public:
    TransformPathConverter(){}
    ~TransformPathConverter();

    /**
     * \param transforms_are_relative if true, the transform represent relative movement.
     *      In this case, starting pose can be set with parameter start_pose.
     * \param start_pose only relevant it transforms_are_relative, and then this is the start pose of the path
     * \param frame_id the transforms (and start_pose) are to be expressed in this frame.
     * \param time time of the frame_id. This is going to be set in all path points, or if path_times is set,
     *  then this applies only to the start point. Default is ros::Time::now().
     * \param path_times if not empty, it has to be the size of transforms. It contains, for each point in
     *      transforms, the time from the start of the trajectory.
     */
    static void convert(const std::vector<geometry_msgs::Transform>& transforms, nav_msgs::Path& path,
        const std::string& frame_id, 
        bool transforms_are_relative, 
        const geometry_msgs::Pose& start_pose=geometry_msgs::Pose(),
        const ros::Time& time=ros::Time::now(), 
        const std::vector<ros::Duration>& path_times=std::vector<ros::Duration>()) {

        path.poses.clear();
        path.header.frame_id=frame_id;
        path.header.stamp=time;

    
        Eigen::Vector3d pose;
        Eigen::Quaterniond orientation;
        
        EigenTransform currTransform;    
        currTransform.setIdentity();

        geometry_msgs::PoseStamped curr;
        curr.header.frame_id=frame_id;
        curr.header.stamp=time;
        
        if (transforms_are_relative) {
            //first point is start pose
            curr.pose=start_pose;
            path.poses.push_back(curr); 
        }

        int i=0;
        for (std::vector<geometry_msgs::Transform>::const_iterator it=transforms.begin(); it!=transforms.end(); ++it) {
            if (transforms_are_relative) {
                //read the transforms from the message
                Eigen::Vector3d trans;
                Eigen::Quaterniond rot;    
                tf::quaternionMsgToEigen (it->rotation, rot);
                tf::vectorMsgToEigen (it->translation, trans);

                currTransform.translate(trans);            
                currTransform.rotate(rot);            

                Eigen::Vector3d nextPos=currTransform.translation(); 
                Eigen::Quaterniond nextOri=Eigen::Quaterniond(currTransform.rotation()); 

                geometry_msgs::Point nextPoint;
                nextPoint.x=nextPos.x();
                nextPoint.y=nextPos.y();
                nextPoint.z=nextPos.z();
                curr.pose.position=nextPoint;

                tf::quaternionEigenToMsg (nextOri,curr.pose.orientation);
            } else {
                curr.pose.position.x=it->translation.x;
                curr.pose.position.y=it->translation.y;
                curr.pose.position.z=it->translation.z;
                curr.pose.orientation.x=it->rotation.x;
                curr.pose.orientation.y=it->rotation.y;
                curr.pose.orientation.z=it->rotation.z;
                curr.pose.orientation.w=it->rotation.w;
            }
            //ROS_INFO_STREAM("Path point: "<<curr.pose.position);    
            if (!path_times.empty()) curr.header.stamp=time+path_times[i];
            path.poses.push_back(curr);    
            ++i;
        }

    }
    
    static void convert(const nav_msgs::Path& path,  std::vector<geometry_msgs::Transform>& transforms,
        bool make_relative_transforms) 
    {
        ROS_ERROR("This is an untested implementation, check it!");

        if (path.poses.empty()) return; 


                
        if (!make_relative_transforms) {
            geometry_msgs::Transform t;
            const geometry_msgs::PoseStamped& p=path.poses.front();
            t.translation.x=p.pose.position.x;
            t.translation.y=p.pose.position.y;
            t.translation.z=p.pose.position.z;
            t.rotation.x=p.pose.orientation.x;
            t.rotation.y=p.pose.orientation.y;
            t.rotation.z=p.pose.orientation.z;
            t.rotation.w=p.pose.orientation.w;
            transforms.push_back(t);
        }
            

        if (make_relative_transforms && (path.poses.size()==1)) {
            transforms.push_back(geometry_msgs::Transform()); //empty transform
            return;
        }
                
        geometry_msgs::PoseStamped lastPose=path.poses.front();

        for (int i=1; i<path.poses.size(); ++i) {
            const geometry_msgs::PoseStamped& p=path.poses[i];
            geometry_msgs::Transform t;
            if (make_relative_transforms) {
                t.translation.x=p.pose.position.x;
                t.translation.y=p.pose.position.y;
                t.translation.z=p.pose.position.z;
                t.rotation.x=p.pose.orientation.x;
                t.rotation.y=p.pose.orientation.y;
                t.rotation.z=p.pose.orientation.z;
                t.rotation.w=p.pose.orientation.w;
            }else{
                t.translation.x=p.pose.position.x - lastPose.pose.position.x;
                t.translation.y=p.pose.position.y - lastPose.pose.position.y;
                t.translation.z=p.pose.position.z - lastPose.pose.position.z;
                Eigen::Quaterniond currOri,lastOri, diff;
                tf::quaternionMsgToEigen (lastPose.pose.orientation, lastOri);
                tf::quaternionMsgToEigen (p.pose.orientation, currOri);
                diff=getRotationFromTo(lastOri,currOri);
                tf::quaternionEigenToMsg(diff,t.rotation);
            }    
            transforms.push_back(t);
            lastPose=p;
        }
    }



    template<typename T>
    static Eigen::Quaternion<T> getRotationFromTo(const Eigen::Quaternion<T>& q1, const Eigen::Quaternion<T>& q2) {
        Eigen::Quaternion<T> ret= q2*q1.inverse();
        ret.normalize();
        return ret;
    }




};

} // end namespace 
