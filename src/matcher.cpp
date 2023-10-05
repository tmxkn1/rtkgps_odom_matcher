#include "matcher.h"

using namespace rtkgps_odom_matcher;

Matcher::Matcher(ros::NodeHandle &node, ros::NodeHandle &privateNode) 
:wheelBufferTime(ros::Time::now())
{
    privateNode.param<std::string>("frame_id", frameId, "gps_map");
    privateNode.param<std::string>("child_frame_id", childFrameId, "map");

    gpggaSub = node.subscribe("nmea/gpgga", 10, &Matcher::gpggaCallback, this);
    gpsOdomSub = node.subscribe("odometry/gps", 10, &Matcher::gpsOdomCallback, this);
    wheelOdomSub = node.subscribe("odometry/wheel", 10, &Matcher::wheelOdomCallback, this);
    ROS_INFO("Matcher initialized.");
}

Matcher::~Matcher()
{}

void Matcher::publishTransform(tf2::Vector3 target[], tf2::Vector3 line[])
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = wheelBufferTime;
    transformStamped.header.frame_id = frameId;
    transformStamped.child_frame_id = childFrameId;
    findTransformation(target, line, transformStamped);
    br.sendTransform(transformStamped);
}

void Matcher::findTransformation(tf2::Vector3 target[], tf2::Vector3 line[], geometry_msgs::TransformStamped &transformStamped)
{
    if (!translationSet)
    {
        translation.x = line[0].x() - target[0].x();
        translation.y = line[0].y() - target[0].y();
        translation.z = line[0].z() - target[0].z();
        translationSet = true;
    }
    transformStamped.transform.translation = translation;
    tf2::Vector3 targetVec = target[1] - target[0];
    tf2::Vector3 lineVec = line[1] - line[0];
    tf2::Quaternion q;
    double yaw = atan2(lineVec.y(), lineVec.x()) - atan2(targetVec.y(), targetVec.x());
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    // ss << yaw;
    // ROS_DEBUG_STREAM(ss.str());
}

void Matcher::gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg)
{
    gpsQauality = msg->gps_qual;
}

void Matcher::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    wheelPositionBuffer = tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
    wheelBufferTime = msg->header.stamp;
    wheelBufferSeq = msg->header.seq;
    wheelOdomPositionInitialized = true;
}

void Matcher::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (gpsQauality != 4)
    {
        ROS_WARN("RTK GPS is not fixed, retaining old transformation.");
        return;
    }
    if (!wheelOdomPositionInitialized)
    {
        ROS_WARN("Wheel odometry is not initialized, waiting for wheel odom.");
        return;
    }
    ros::Duration d = msg->header.stamp - wheelBufferTime;
    if ( d > ros::Duration(0.5) || d < ros::Duration(-0.5))
    {
        ROS_WARN_STREAM(counter << ": GWheel odom ["<< wheelBufferSeq <<"] is over 0.1 s older, retaining old transformation.");
        // ROS_DEBUG_STREAM(msg->header.seq << "," <<wheelBufferSeq << "," << std::setprecision(15) << msg->header.stamp.toSec() << "," << std::setprecision(15) << wheelBufferTime.toSec() << ",nan");
        return;
    }
    if (!gpsOdomPositionInitialized)
    {
        gpsPosition = tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
        gpsOdomPositionInitialized = true;
        wheelPosition = tf2::Vector3(wheelPositionBuffer.x(), wheelPositionBuffer.y(), 0);
        return;
    }

    tf2::Vector3 gpsPositionBuffer = tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
    if (tf2::tf2Distance(gpsPosition, gpsPositionBuffer) < 0.01)
    {
        ROS_WARN_STREAM("GPS position change is " << tf2::tf2Distance(gpsPosition, gpsPositionBuffer) << ", less than 0.01 m, retaining old transformation.");
        return;
    }
    tf2::Vector3 target[] = {gpsPosition, gpsPositionBuffer};
    tf2::Vector3 line[] = {wheelPosition, wheelPositionBuffer};
    // ss.str("");
    // ss << msg->header.seq << "," <<wheelBufferSeq << "," << std::setprecision(15) << msg->header.stamp.toSec() << "," << wheelBufferTime.toSec() << ",";
    publishTransform(target, line);

    gpsPosition = tf2::Vector3(gpsPositionBuffer.x(), gpsPositionBuffer.y(), 0);
    wheelPosition = tf2::Vector3(wheelPositionBuffer.x(), wheelPositionBuffer.y(), 0);
}
