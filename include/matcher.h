#pragma once

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <nmea_msgs/Gpgga.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rtkgps_odom_matcher
{
    
class Matcher
{
public:
    Matcher(ros::NodeHandle &node, ros::NodeHandle &privateNode);
    ~Matcher();

private:
    std::string frameId;
    std::string childFrameId;
    uint32_t gpsQauality = 0;
    bool gpsOdomPositionInitialized = false;
    bool wheelOdomPositionInitialized = false;
    bool translationSet = false;
    geometry_msgs::Vector3 translation;
    tf2::Vector3 gpsPosition;
    tf2::Vector3 wheelPosition;
    tf2::Vector3 wheelPositionBuffer;
    ros::Time wheelBufferTime;
    uint32_t wheelBufferSeq = 0;
    uint32_t counter=0;

    ros::Subscriber gpggaSub;
    ros::Subscriber gpsOdomSub;
    ros::Subscriber wheelOdomSub;
    std::stringstream ss;

    void publishTransform(tf2::Vector3 target[], tf2::Vector3 line[]);
    void findTransformation(tf2::Vector3 target[], tf2::Vector3 line[], geometry_msgs::TransformStamped &transformStamped);
    void gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg);
    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void updateMovingAvg(double newval);
};
}
