#pragma once

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <nmea_msgs/Gpgga.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "libicp/matrix.h"
#include "libicp/icpPointToPoint.h"
#include "odomDataStore.h"

namespace rtkgps_odom_matcher
{

class Matcher
{
public:
    Matcher(ros::NodeHandle &node, ros::NodeHandle &privateNode);
    ~Matcher();

private:
    std::string worldFrameId;
    std::string childFrameId;
    int gpsOdomMaxBufferSize = 500;
    int wheelOdomMaxBufferSize = 500;
    double gpsOdomTolerance = 0.1;
    double wheelOdomTolerance = 0.01;
    bool useFloatingRTK = true;
    double updateRate = 1;
    int minRequiredBufferSize = 5;
    double timeSeekingTolerance = 0.5;

    OdomDataStore *gpsDs;
    OdomDataStore *wheelDs;
    std::deque<int> wheelDsIndex;
    uint32_t gpsQauality = 0;
    bool isTransformReady = false;
    geometry_msgs::TransformStamped transformStamped;
    Matrix R;
    Matrix T;
    
    ros::Subscriber gpggaSub;
    ros::Subscriber gpsOdomSub;
    ros::Subscriber wheelOdomSub;
    ros::Timer periodicUpdateTimer_;

    void findTransform();
    void removeOldestDataBlock();
    
    void gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg);
    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void periodicUpdate(const ros::TimerEvent& event);
    
    static double getDistance(double x1, double y1, double x2, double y2);
};
}
