#include <object_moveit/MoveItHelpers.h>
#include <shape_tools/solid_primitive_dims.h>

using object_moveit::MoveItHelpers;

std::ostream& operator<<(std::ostream& o, const Eigen::Quaterniond& q) {
    o<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w();
    return o;
}
std::ostream& operator<<(std::ostream& o, const Eigen::Vector3d& v) {
    o<<v.x()<<","<<v.y()<<","<<v.z();
    return o;
}


shape_msgs::SolidPrimitive getCone(const double& height, const double& radius){
    shape_msgs::SolidPrimitive bv;
    bv.type = shape_msgs::SolidPrimitive::CONE;
    bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;
    bv.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius;
    return bv;

}

shape_msgs::SolidPrimitive getCylinder(const double& height, const double& radius){
    shape_msgs::SolidPrimitive bv;
    bv.type = shape_msgs::SolidPrimitive::CYLINDER;
    bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
    bv.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
    return bv;

}

shape_msgs::SolidPrimitive getBox(const double& x,const  double& y,const  double& z){
    shape_msgs::SolidPrimitive bv;
    bv.type = shape_msgs::SolidPrimitive::BOX;
    bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_X] = x;
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y;
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z;
    return bv;

}
shape_msgs::SolidPrimitive getSphere(const double& radius){
    shape_msgs::SolidPrimitive bv;
    bv.type = shape_msgs::SolidPrimitive::SPHERE;
    bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
    bv.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;
    return bv;
}

moveit_msgs::PositionConstraint object_moveit::MoveItHelpers::getBoxConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& boxOrigin, const double& x, const double& y, const double& z) {
    
    moveit_msgs::PositionConstraint pc;
    pc.link_name = link_name;
    pc.target_point_offset.x = 0;
    pc.target_point_offset.y = 0;
    pc.target_point_offset.z = 0;
    
    pc.header=boxOrigin.header;
    
    pc.constraint_region.primitives.resize(1);
    shape_msgs::SolidPrimitive &bv = pc.constraint_region.primitives[0];
    bv=getBox(x,y,z);

    pc.constraint_region.primitive_poses.resize(1);
    geometry_msgs::Pose& pose=pc.constraint_region.primitive_poses[0];
    pose=boxOrigin.pose;    

    pc.weight=1.0;

    return pc;
}

#if 0
bool object_moveit::MoveItHelpers::getCylinderConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to,
    const double& radius, moveit_msgs::PositionConstraint& result) {

    geometry_msgs::PoseStamped target;
    if (CapabilityHelpers::Singleton()->transformPose(to,from.header.frame_id,target,-1,true)<0) {
        ROS_ERROR("Can't transform between poses");
        return false;
    }

    Eigen::Vector3d _from,_to;
    tf::pointMsgToEigen(from.pose.position, _from);
    tf::pointMsgToEigen(target.pose.position, _to);

    Eigen::Vector3d _diff=_to-_from;
    geometry_msgs::Vector3 diff;
    tf::vectorEigenToMsg(_diff,diff);


    result=getCylinderConstraint(link_name,from,diff,radius);

    return true;

}
#endif

object_moveit::MoveItHelpers::SolidPrimitivePtr object_moveit::MoveItHelpers::getCylinderBV(
        const Eigen::Vector3d& _fromPose, const Eigen::Quaterniond& _fromOrientation,
        const Eigen::Vector3d& _direction, const double& radius,
        Eigen::Vector3d& bv_pose, Eigen::Quaterniond& bv_orientation) {

    if (_direction.norm()<1e-06) {
        ROS_ERROR("Cone direction can't be 0 length!");
        return SolidPrimitivePtr((shape_msgs::SolidPrimitive*)NULL);
    }

    bv_pose=_fromPose;
    bv_orientation=_fromOrientation;

    // because cones and cylinders are oriented around z, we first have to
    // rotate the z-axis in <from> such that it points to <dir>
    Eigen::Vector3d _z= bv_orientation * Eigen::Vector3d(0,0,1);
    Eigen::Quaterniond quat;
    quat.setFromTwoVectors(_z,_direction);
    quat.normalize();
    bv_orientation*=quat;
    
    float height=_direction.norm();

    //ROS_INFO_STREAM("Adding movement along CYL: "<<_fromPose<<" -- > "
    //  <<_direction<<" (length "<<height<<"), radius "<<radius);

    SolidPrimitivePtr bv(new shape_msgs::SolidPrimitive());
    *bv = getCylinder(height,radius);

    return bv;
}

/*
moveit_msgs::PositionConstraint object_moveit::MoveItHelpers::getCylinderConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& from, const geometry_msgs::Vector3& dir,
        const double& radius)
{
    moveit_msgs::PositionConstraint pc;

    ROS_ERROR("Change getCylinderConstraint");
    
#if 0   //XXX change this to call betCylinderBV
    Eigen::Vector3d _from,_dir;
    tf::pointMsgToEigen(from.pose.position, _from);
    tf::vectorMsgToEigen(dir, _dir);

    Eigen::Quaterniond _ori;
    tf::quaternionMsgToEigen(from.pose.orientation, _ori);


    //because cones and cylinders are oriented around z, we first have to rotate the z-axis in <from> such that it points to <dir>
    Eigen::Vector3d _z= _ori * Eigen::Vector3d(0,0,1);

    Eigen::Quaterniond quat;
    quat.setFromTwoVectors(_z,_dir);
    quat.normalize();
    _ori*=quat;
    
    geometry_msgs::PoseStamped cylOrigin=from;
    tf::quaternionEigenToMsg(_ori,cylOrigin.pose.orientation);


    float height=_dir.norm();

    ROS_INFO_STREAM("Adding movement along CYL: "<<cylOrigin<<" -- > "<<_dir<<" (length "<<height<<"), radius "<<radius);


    pc.link_name = link_name;
    pc.target_point_offset.x = 0;
    pc.target_point_offset.y = 0;
    pc.target_point_offset.z = 0;
    
    pc.header=from.header;
    pc.weight=1.0;
    
    pc.constraint_region.primitives.resize(1);
    shape_msgs::SolidPrimitive &bv = pc.constraint_region.primitives[0];
    bv=getCylinder(height,radius);

    pc.constraint_region.primitive_poses.resize(1);
    geometry_msgs::Pose& pose=pc.constraint_region.primitive_poses[0];
    pose=cylOrigin.pose;    
#endif

    return pc;
}
*/

object_moveit::MoveItHelpers::SolidPrimitivePtr object_moveit::MoveItHelpers::getConeBV(
        const Eigen::Vector3d& _fromPose, const Eigen::Quaterniond& _fromOrientation,
        const Eigen::Vector3d& _direction, const double& radius_start, const double& radius_end, 
        Eigen::Vector3d& bv_pose, Eigen::Quaterniond& bv_orientation) {

    if (_direction.norm()<1e-06) {
        ROS_ERROR("Cone direction can't be 0 length!");
        return SolidPrimitivePtr((shape_msgs::SolidPrimitive*)NULL);
    }

    if (fabs(radius_start-radius_end) < 1e-06) {
        ROS_INFO("Radius are equal, hence this is a cylinder");
        return getCylinderBV(_fromPose, _fromOrientation, _direction, radius_start, bv_pose, bv_orientation);
    }
    
    bool startIsTip = radius_start < radius_end;

    bv_pose=_fromPose;
    bv_orientation=_fromOrientation;

    
    ROS_INFO_STREAM("Getting CONE for inital pose "<<bv_pose<<", ori "<<bv_orientation
            <<" (cone direction "<<_direction<<"), radius "<<radius_start<<".."<<radius_end);

    // Because cone tip points in +z, we first have to rotate the z-axis in <_fromOrientation>
    // such that it points along <_direction>. If the start radius is the tip, then +z has 
    // to point in -<dir>, because the tip of the cone is in positive direction.
    Eigen::Vector3d _z= bv_orientation * Eigen::Vector3d(0,0,1);

    Eigen::Quaterniond quat;
    Eigen::Vector3d _negDir=-_direction;
    quat.setFromTwoVectors(_z, startIsTip ? _negDir : _direction);
    quat.normalize();
    bv_orientation*=quat;
    
    ROS_INFO_STREAM("Rotated orientation: "<<bv_orientation);


    float dist=_direction.norm();
    // now, we have to re-calculate the origin of the cone, which is NOT <from>, but goes back along
    // -<dir> such that the cone has radius <radius_end> at <from>.
    float addDist = 0;
    if (startIsTip) addDist = (radius_start * dist) / (radius_end - radius_start); 
    else addDist = (radius_end * dist) / (radius_start - radius_end); 

    
    if (startIsTip) {
        //ROS_INFO("Have to displace the cone!");    
        //because the cone is inverted we have to dispace the origin of the cone.
        Eigen::Vector3d addDir=_direction;
        addDir.normalize();
        addDir*=addDist;

        Eigen::Vector3d addToFrom=_direction+addDir;

        // the pose of the cone has to start at the <from>+<dir> point, because the cone
        // points in +z, and we have it's positive z-axis oriented along -dir
        bv_pose+=addToFrom;
    }

    //now, we have to transform the cone origin orientation so that the cylinder will point along z-axis
    float height = dist+addDist;
    float radius = startIsTip ? radius_end : radius_start;

    //ROS_INFO_STREAM("Got CONE: pose "<<bv_pose<<", ori "<<bv_orientation<<" (length "<<height<<"), radius "<<radius);

    SolidPrimitivePtr bv(new shape_msgs::SolidPrimitive());
    *bv=getCone(height, radius);

    //ROS_INFO_STREAM("Cone: "<<std::endl<<*bv);

    return bv;
}


/*moveit_msgs::PositionConstraint object_moveit::MoveItHelpers::getConeConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& from, const geometry_msgs::Vector3& dir,
        const double& radius_start, const double& radius_end)
{
    moveit_msgs::PositionConstraint pc;
    ROS_ERROR("CHange me");
#if 0 //XXX change so that it uses getConeBV()
    bool startIsTip = radius_start<radius_end;
    
    Eigen::Vector3d _from,_dir;
    tf::pointMsgToEigen(from.pose.position, _from);
    tf::vectorMsgToEigen(dir, _dir);

    Eigen::Quaterniond _ori;
    tf::quaternionMsgToEigen(from.pose.orientation, _ori);


    //because cone tip points in +z, we first have to rotate the z-axis in <from> such that it points along <dir>.
    //if the start radius is the tip, then +z has to point in -<dir>, because the tip of the cone is in positive direction.
    Eigen::Vector3d _z= _ori * Eigen::Vector3d(0,0,1);

    Eigen::Quaterniond quat;
    Eigen::Vector3d _negDir=-_dir;
    quat.setFromTwoVectors(_z, startIsTip ? _negDir : _dir);
    quat.normalize();
    _ori*=quat;


    float dist=_dir.norm();
    // now, we have to re-calculate the origin of the cone, which is NOT <from>, but goes back along 
    // -<dir> such that the cone has radius <radius_end> at <from>.
    float addDist = 0;
    if (startIsTip) addDist = (radius_start * dist) / (radius_end - radius_start); 
    else addDist = (radius_end * dist) / (radius_start - radius_end); 

    geometry_msgs::PoseStamped coneOrigin=from;
    
    if (startIsTip) {    
        //because the cone is inverted we have to dispace the origin of the cone.
        Eigen::Vector3d addDir=_dir;
        addDir.normalize();
        addDir*=addDist;

        Eigen::Vector3d addToFrom=_dir+addDir;

        // the pose of the cone has to start at the <from>+<dir> point, because the cone points in +z,
        // and we have it's positive z-axis oriented along -dir
        coneOrigin.pose.position.x+=addToFrom.x();
        coneOrigin.pose.position.y+=addToFrom.y();
        coneOrigin.pose.position.z+=addToFrom.z();
    }

    //now, we have to transform the coneOrigin orientation so that the cylinder will point along z-axis
    tf::quaternionEigenToMsg(_ori,coneOrigin.pose.orientation);

    float height = dist+addDist;
    float radius = startIsTip ? radius_end : radius_start;

    ROS_INFO_STREAM("Adding movement along CONE: "<<coneOrigin<<" -- > "<<dir<<" (length "<<height<<"), radius "<<radius);


    pc.link_name = link_name;
    pc.target_point_offset.x = 0;
    pc.target_point_offset.y = 0;
    pc.target_point_offset.z = 0;
    pc.weight=1.0;
    
    pc.header=from.header;
    
    pc.constraint_region.primitives.resize(1);
    shape_msgs::SolidPrimitive &bv = pc.constraint_region.primitives[0];
    bv=getCone(height, radius);

    pc.constraint_region.primitive_poses.resize(1);
    geometry_msgs::Pose& pose=pc.constraint_region.primitive_poses[0];
    pose=coneOrigin.pose;    

#endif
    return pc;
}
*/

moveit_msgs::Constraints object_moveit::MoveItHelpers::getJointConstraint(const std::string& link_name,
        const sensor_msgs::JointState& js, const float& joint_tolerance){
    
    moveit_msgs::Constraints c;
    for (int i=0; i<js.name.size(); ++i){
        moveit_msgs::JointConstraint jc;
        jc.joint_name=js.name[i];
        jc.position=js.position[i];
        jc.tolerance_above=joint_tolerance;
        jc.tolerance_below=joint_tolerance;
        jc.weight=1.0;
        c.joint_constraints.push_back(jc);
    }
    return c;

}


moveit_msgs::OrientationConstraint object_moveit::MoveItHelpers::getOrientationConstraint(const std::string& link_name,
        const geometry_msgs::QuaternionStamped& quat, 
    const float& x_tolerance,
    const float& y_tolerance,
    const float& z_tolerance) { 
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = link_name;
    ocm.header = quat.header;
    ocm.orientation = quat.quaternion;
    ocm.absolute_x_axis_tolerance = x_tolerance;
    ocm.absolute_y_axis_tolerance = y_tolerance;
    ocm.absolute_z_axis_tolerance = z_tolerance;
    ocm.weight = 1.0;
    return ocm;
}

object_moveit::MoveItHelpers::PositionConstraintPtr object_moveit::MoveItHelpers::getSpherePoseConstraint(
        const std::string &link_name, 
        const geometry_msgs::PoseStamped &target_pose1, 
        float maxArmReachDistance) {

    //ROS_INFO_STREAM("Combined path: "<<std::endl<<target_pose1<<std::endl<<target_pose2);
    //ROS_INFO_STREAM("Combined path: "<<_target_pose1<<", dir "<<_target1_to_target2);

    PositionConstraintPtr pc(new moveit_msgs::PositionConstraint());
    pc->link_name = link_name;
    pc->weight=1.0;
    pc->target_point_offset.x = 0;
    pc->target_point_offset.y = 0;
    pc->target_point_offset.z = 0;
    pc->header=target_pose1.header;
    
    pc->constraint_region.primitives.resize(1);
    pc->constraint_region.primitive_poses.resize(1);
    
    shape_msgs::SolidPrimitive &bv1 = pc->constraint_region.primitives[0];
    geometry_msgs::Pose& pose1=pc->constraint_region.primitive_poses[0];

    geometry_msgs::Pose sphereOrigin=target_pose1.pose;
    pose1=sphereOrigin;    
    bv1=getSphere(maxArmReachDistance);
    return pc;
}

#if 0
object_moveit::MoveItHelpers::PositionConstraintPtr object_moveit::MoveItHelpers::getCombinedPoseConstraint(
        const std::string &link_name, 
        const geometry_msgs::PoseStamped &target_pose1, 
        float tolerance_pos1, 
        const geometry_msgs::PoseStamped& target_pose2, 
        float tolerance_pos2, 
        float maxArmReachDistance) {


    //first, define the sphere
    geometry_msgs::Pose target1_to_target2;
    int rel=CapabilityHelpers::Singleton()->relativePose(target_pose1,target_pose2,target1_to_target2,true,0.2,true);

    if (rel < 0) {
        ROS_ERROR("Could not get transform between frames %s and %s: error %i",target_pose1.header.frame_id.c_str(),target_pose2.header.frame_id.c_str(),rel);
        return PositionConstraintPtr((moveit_msgs::PositionConstraint*)NULL);     
    }

    Eigen::Vector3d _target1_to_target2;
    tf::pointMsgToEigen(target1_to_target2.position,_target1_to_target2);

    Eigen::Vector3d _target_pose1;
    tf::pointMsgToEigen(target_pose1.pose.position,_target_pose1);

    Eigen::Quaterniond _target1_orientation;
    tf::quaternionMsgToEigen(target_pose1.pose.orientation, _target1_orientation);

    //ROS_INFO_STREAM("Combined path: "<<std::endl<<target_pose1<<std::endl<<target_pose2);
    //ROS_INFO_STREAM("Combined path: "<<_target_pose1<<", dir "<<_target1_to_target2);


    Eigen::Vector3d _target1_to_target2_orig=_target1_to_target2; //backup original vector
    _target1_to_target2.normalize();

    Eigen::Vector3d _maxArmReachCenter=_target_pose1 + _target1_to_target2* (-maxArmReachDistance);
    
    PositionConstraintPtr pc(new moveit_msgs::PositionConstraint());
    pc->link_name = link_name;
    pc->weight=1.0;
    pc->target_point_offset.x = 0;
    pc->target_point_offset.y = 0;
    pc->target_point_offset.z = 0;
    pc->header=target_pose1.header;
    
    pc->constraint_region.primitives.resize(2);
    pc->constraint_region.primitive_poses.resize(2);
    
    //--- Define the box ---
    shape_msgs::SolidPrimitive &bv1 = pc->constraint_region.primitives[0];
    shape_msgs::SolidPrimitive &bv2 = pc->constraint_region.primitives[1];
    geometry_msgs::Pose& pose1=pc->constraint_region.primitive_poses[0];
    geometry_msgs::Pose& pose2=pc->constraint_region.primitive_poses[1];

    float doubleArmReach=2*maxArmReachDistance;
    bv1=getBox(doubleArmReach,doubleArmReach,doubleArmReach);

    // orient the box such that it sits flat on the cone base.
    // We specify everyting in target_pose1 frame, so rotate this frame
    // such that it's +z axis aligns with _target1_to_target2
    Eigen::Quaterniond _ori1=_target1_orientation;
    Eigen::Vector3d _z1= _ori1 * Eigen::Vector3d(0,0,1);
    Eigen::Quaterniond quat;
    quat.setFromTwoVectors(_z1, _target1_to_target2);
    _ori1*=quat;
    geometry_msgs::Pose boxOrigin=target_pose1.pose;
    tf::quaternionEigenToMsg(_ori1,boxOrigin.orientation);
    boxOrigin.position.x=_maxArmReachCenter.x();
    boxOrigin.position.y=_maxArmReachCenter.y();
    boxOrigin.position.z=_maxArmReachCenter.z();
    pose1=boxOrigin;    


    //--- Define the cone ---
    Eigen::Vector3d cone_pose;
    Eigen::Quaterniond cone_orientation;
    SolidPrimitivePtr cone=getConeBV(_target_pose1, _target1_orientation, _target1_to_target2_orig,
            tolerance_pos1, tolerance_pos2, cone_pose, cone_orientation);
    if (!cone.get()){
        ROS_ERROR("Could not get cone BoundingVolume");
        return PositionConstraintPtr((moveit_msgs::PositionConstraint*)NULL);     
    }
    

    geometry_msgs::Pose coneOrigin=target_pose1.pose; 
    tf::pointEigenToMsg(cone_pose,coneOrigin.position);
    tf::quaternionEigenToMsg(cone_orientation,coneOrigin.orientation);
    bv2=*cone;
    pose2=coneOrigin;    

    ROS_INFO_STREAM("Cone origin and orientation"<<std::endl<<bv2); ROS_INFO_STREAM(pose2);

    return pc;
}
#endif

moveit_msgs::Constraints object_moveit::MoveItHelpers::getPoseConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped &pose, double tolerance_pos, double tolerance_angle, int type) {

    moveit_msgs::Constraints goal;

    if (type<=1) {    
        goal.position_constraints.resize(1);
        moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
        pcm.link_name = link_name;
        pcm.target_point_offset.x = 0;
        pcm.target_point_offset.y = 0;
        pcm.target_point_offset.z = 0;
        pcm.constraint_region.primitives.resize(1);
        shape_msgs::SolidPrimitive &bv = pcm.constraint_region.primitives[0];
        bv=getSphere(tolerance_pos);

        pcm.header = pose.header;
        pcm.constraint_region.primitive_poses.resize(1);
        pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

        // orientation of constraint region does not affect anything, since it is a sphere
        pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
        pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
        pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
        pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
        pcm.weight = 1.0;
    }
    
    if ((type==1) || (type==2)){
        goal.orientation_constraints.resize(1);
        moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
        ocm.link_name = link_name;
        ocm.header = pose.header;
        ocm.orientation = pose.pose.orientation;
        ocm.absolute_x_axis_tolerance = tolerance_angle;
        ocm.absolute_y_axis_tolerance = tolerance_angle;
        ocm.absolute_z_axis_tolerance = tolerance_angle;
        ocm.weight = 1.0;
    }/*else{
        ROS_WARN("An unsupported constraint type was passed into object_moveit::MoveItHelpers::getPoseConstraint: %i",type);
    }*/
    return goal;
}
