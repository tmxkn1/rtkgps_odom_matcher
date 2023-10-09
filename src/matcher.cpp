#include "matcher.h"

using namespace rtkgps_odom_matcher;

Matcher::Matcher(ros::NodeHandle &node, ros::NodeHandle &privateNode)
: R(Matrix::eye(2)), T(Matrix::zeros(2, 1))
{
    privateNode.param<std::string>("frame_id", frameId, "gps_map");
    privateNode.param<std::string>("child_frame_id", childFrameId, "map");

    gpggaSub = node.subscribe("nmea/gpgga", 10, &Matcher::gpggaCallback, this);
    gpsOdomSub = node.subscribe("odometry/gps", 10, &Matcher::gpsOdomCallback, this);
    wheelOdomSub = node.subscribe("odometry/wheel", 10, &Matcher::wheelOdomCallback, this);
    
    gpsDs = new OdomDataStore(100);
    wheelDs = new OdomDataStore(100);
    minRequiredBufferSize = 5;
    gpsQauality = 4;

    ROS_INFO("Matcher initialized.");
}

Matcher::~Matcher()
{}

void Matcher::publishTransform()
{
    if (gpsDs->size() < minRequiredBufferSize)
    {
        ROS_WARN_STREAM("GPS odom buffer size is " << gpsDs->size() << ", less than " << minRequiredBufferSize << ". Waiting for more data...");
        return;
    }
    if (wheelDs->size() < minRequiredBufferSize)
    {
        ROS_WARN_STREAM("Wheel odom buffer size is " << wheelDs->size() << ", less than " << minRequiredBufferSize << ". Waiting for more data...");
        return;
    }

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time(wheelDs->getTimeBack(0));
    transformStamped.header.frame_id = frameId;
    transformStamped.child_frame_id = childFrameId;
    findTransform(transformStamped);
    br.sendTransform(transformStamped);
}

void Matcher::findTransform(geometry_msgs::TransformStamped &transformStamped)
{
    ROS_INFO_STREAM("Finding transform from wheel odom to GPS odom..." );
    ros::Time startTime = ros::Time::now();
    double wheelPos[wheelDs->size()*2], gpsPos[gpsDs->size()*2];
    wheelDs->getPosition(wheelPos);
    gpsDs->getPosition(gpsPos);
    IcpPointToPoint icp(wheelPos, wheelDs->size(), 2);
    icp.setMaxIterations(10);
    icp.setMinDeltaParam(0.01);
    icp.fit(gpsPos, gpsDs->size(), R, T, -1);

    transformStamped.transform.translation.x = T.val[0][0];
    transformStamped.transform.translation.y = T.val[1][0];
    double yaw = atan2(R.val[1][0], R.val[0][0]);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    ros::Time endTime = ros::Time::now();
    ROS_INFO_STREAM("Found transform from wheel odom to GPS odom in " << (endTime - startTime).toSec() << " s, with points " << gpsDs->size() << " & " << wheelDs->size() << ". Yaw is: " << yaw << ".");
}

void Matcher::gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg)
{
    gpsQauality = msg->gps_qual;
}

void Matcher::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (wheelDs->size() == 0)
    {
        assert(wheelDs->push(msg));
        return;
    }
    
    double prevPos[2];
    wheelDs->getPositionBack(0, prevPos);
    if (getDistance(prevPos[0], prevPos[1], msg->pose.pose.position.x, msg->pose.pose.position.y) < wheelOdomTolerance)
    {
        ROS_DEBUG_STREAM("Wheel Odom change is less than " << wheelOdomTolerance << " m. Skipping data.");
        return;
    }
    ROS_INFO("Wheel odom received.");
    ROS_DEBUG_STREAM("[msg] pose.x: " << msg->pose.pose.position.x << ", pose.y: " << msg->pose.pose.position.y);
    ROS_DEBUG_STREAM("[prevmsg] pose.x: " << prevPos[0] << ", pose.y: " << prevPos[1] << ".");
    if (wheelDs->push(msg))
        return;
    ROS_WARN("Wheel odom buffer overflow, removing old data.");
    removeOldestDataBlock();
    assert(wheelDs->push(msg));
}

void Matcher::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (gpsQauality != 4 && !(useFloatingRTK && gpsQauality == 5))
    {
        ROS_WARN("GPS quality insufficient, skipping data.");
        return;
    }
    if (gpsDs->size() == 0)
    {
        assert(gpsDs->push(msg));
        return;
    }
    
    double prevPos[2];
    gpsDs->getPositionBack(0, prevPos);
    if (getDistance(prevPos[0], prevPos[1], msg->pose.pose.position.x, msg->pose.pose.position.y) < gpsOdomTolerance)
    {
        ROS_DEBUG_STREAM("GPS position change is less than " << gpsOdomTolerance << " m. Skipping data.");
        return;
    }

    if (!gpsDs->push(msg))
    {
        ROS_INFO("GPS odom buffer overflow, clearing buffer.");
        removeOldestDataBlock();
        assert(gpsDs->push(msg));
    }
    // double d = abs(gpsDs->getTime(0 - 1) - wheelDs->getTime(0 - 1));
    // if ( d > 0.5 )
    // {
    //     ROS_WARN_STREAM("GWheel odom is over 0.5 s older, retaining old transformation.");
    //     // ROS_DEBUG_STREAM(msg->header.seq << "," <<wheelBufferSeq << "," << std::setprecision(15) << msg->header.stamp.toSec() << "," << std::setprecision(15) << wheelBufferTime.toSec() << ",nan");
    //     return;
    // }
    
    // ss.str("");
    // ss << msg->header.seq << "," <<wheelBufferSeq << "," << std::setprecision(15) << msg->header.stamp.toSec() << "," << wheelBufferTime.toSec() << ",";
    publishTransform();
}

void Matcher::removeOldestDataBlock()
{
    // GPS odom is published less frequently than Wheel odom, so we use the i-th 
    // oldest GPS data as a reference to locate the wheel odom data published
    // around the similar time, and remove all data before that.
    for (int i = 1; i < gpsDs->size(); i++)
    {
        double gpsTime = gpsDs->getTime(i);
        int index = wheelDs->seekTime(gpsTime, true, timeSeekingTolerance);
        if (index == 0)
            continue;
        if (index < 0)
        {
            ROS_WARN_STREAM("No wheel odom data found around GPS odom data at " << gpsTime << ". Searching for newer time stamp...");
            continue;
        }
        wheelDs->discardFront(index);
        gpsDs->discardFront(i);
        ROS_INFO_STREAM("Removed " << index << " oldest wheel odom data and " << i << " oldest GPS odom data.");
        ROS_INFO_STREAM("New data sizes: wheel odom: " << wheelDs->size() << ", GPS odom: " << gpsDs->size() << ".");
        return;
    }
    ROS_WARN_STREAM("No wheel odom data found around any GPS odom data. Clearing all data...");
    wheelDs->clear();
    gpsDs->clear();
}

double Matcher::getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}