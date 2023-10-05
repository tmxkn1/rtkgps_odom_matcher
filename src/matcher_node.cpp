#include <ros/ros.h>
#include "matcher.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "matcher_node");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    rtkgps_odom_matcher::Matcher matcher(node, privateNode);

    ros::spin();
}
