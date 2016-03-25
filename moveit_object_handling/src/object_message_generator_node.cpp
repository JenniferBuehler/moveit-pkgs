#include <moveit_object_handling/ObjectMessageGenerator.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_message_generator");
    ros::NodeHandle node_priv("~");
    ros::NodeHandle node("");
    moveit_object_handling::ObjectMessageGenerator generator(node_priv, node);

    ros::spin();

    return 0;
}
