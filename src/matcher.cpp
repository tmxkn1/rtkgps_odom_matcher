#include "matcher.h"

using namespace rtkgps_odom_matcher;

Matcher::Matcher(ros::NodeHandle &node, ros::NodeHandle &privateNode)
    : R(Matrix::eye(2)), T(Matrix::zeros(2, 1))
{
    privateNode.param<std::string>("world_frame_id", worldFrameId, "sat_map_origin");
    privateNode.param<std::string>("child_frame_id", childFrameId, "map");
    privateNode.param<int>("gps_odom_max_buffer_size", gpsOdomMaxBufferSize, 500);
    privateNode.param<int>("wheel_odom_max_buffer_size", wheelOdomMaxBufferSize, 500);
    privateNode.param<double>("gps_odom_tolerance", gpsOdomTolerance, 0.1);
    privateNode.param<double>("wheel_odom_tolerance", wheelOdomTolerance, 0.01);
    privateNode.param<bool>("use_floating_rtk", useFloatingRTK, true);
    privateNode.param<double>("update_rate", updateRate, 1.0);
    privateNode.param<int>("min_required_buffer_size", minRequiredBufferSize, 5);
    privateNode.param<bool>("start_immediately", poseMatchStarted, false);
    ROS_INFO_STREAM("world_frame_id: " << worldFrameId);
    if (minRequiredBufferSize < 5)
    {
        ROS_WARN_STREAM("min_required_buffer_size is less than 5, setting to 5.");
        minRequiredBufferSize = 5;
    }
    privateNode.param<double>("time_seeking_tolerance", timeSeekingTolerance, 0.5);

    gpsDs = new OdomDataStore(gpsOdomMaxBufferSize);
    wheelDs = new OdomDataStore(wheelOdomMaxBufferSize);
    wheelDsIndex = std::deque<int>();
    minRequiredBufferSize = 5;
    gpsQauality = 4;
    isTransformReady = false;

    transformStamped.header.frame_id = worldFrameId;
    transformStamped.child_frame_id = childFrameId;
    transformStamped.transform.translation = tf2::toMsg(tf2::Vector3(0, 0, 0));
    transformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

    gpggaSub = node.subscribe("nmea/gpgga", 10, &Matcher::gpggaCallback, this);
    gpsOdomSub = node.subscribe("odometry/gps", 10, &Matcher::gpsOdomCallback, this);
    wheelOdomSub = node.subscribe("odometry/wheel", 10, &Matcher::wheelOdomCallback, this);
    periodicUpdateTimer_ = node.createTimer(ros::Duration(1./updateRate), &Matcher::periodicUpdate, this);

    startMatchSrv = node.advertiseService("start_match", &Matcher::startMatchCallback, this);
    stopMatchSrv = node.advertiseService("stop_match", &Matcher::stopMatchCallback, this);
    sendPoseEstSrv = node.advertiseService("send_pose_est", &Matcher::sendPoseEstCallback, this);
    flipMatchSrv = node.advertiseService("flip_match", &Matcher::flipMatchCallback, this);

    ROS_INFO("rtkgps_odom_matcher initialized.");
}

Matcher::~Matcher()
{
}

void Matcher::findTransform()
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

    ros::Time startTime = ros::Time::now();

    double wheelPos[wheelDs->size() * 2], gpsPos[gpsDs->size() * 2];
    wheelDs->getPosition(wheelPos);
    gpsDs->getPosition(gpsPos);

    IcpPointToPoint icp(wheelPos, wheelDs->size(), 2);
    icp.setMaxIterations(10);
    icp.setMinDeltaParam(0.01);

    const std::lock_guard<std::mutex> lock(mutex);
    icp.fit(gpsPos, gpsDs->size(), R, T, -1);
    R.val[0][0] = flipMatch * R.val[0][0];
    R.val[0][1] = flipMatch * R.val[0][1];
    R.val[1][0] = flipMatch * R.val[1][0];
    R.val[1][1] = flipMatch * R.val[1][1];
    flipMatch = 1;

    tf2::Matrix3x3 RM;
    RM.setValue(R.val[0][0], R.val[0][1], 0,
                R.val[1][0], R.val[1][1], 0,
                0, 0, 1);
    tf2::Quaternion q;
    RM.getRotation(q);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.translation.x = T.val[0][0];
    transformStamped.transform.translation.y = T.val[1][0];

    isTransformReady = true;

    mutex.unlock();

    ros::Time endTime = ros::Time::now();
    ROS_DEBUG_STREAM("Found transform from wheel odom to GPS odom in " << (endTime - startTime).toSec() 
    << " s, with " << gpsDs->size() << " GPS Odom data & " << wheelDs->size() << " Wheel Odom data.");
}

void Matcher::removeOldestDataBlock()
{
    // GPS odom is published less frequently than Wheel odom, so we use the i-th
    // oldest GPS data as a reference to locate the wheel odom data published
    // around the similar time, and remove all data before that.
    ros::Time start = ros::Time::now();
    for (int i = 0; i < gpsDs->size(); i++)
    {
        double gpsTime = gpsDs->getTime(i);
        int index = wheelDs->seekTime(gpsTime, timeSeekingTolerance);
        if (index <= 0)
        {
            continue;
        }
        wheelDs->discardFront(index);
        gpsDs->discardFront(i);
        ros::Time end = ros::Time::now();
        ROS_DEBUG_STREAM("Removed " << index << " oldest wheel odom data and " << i << " oldest GPS odom data. Used " << (end - start).toSec() << " s.");
        ROS_DEBUG_STREAM("New data sizes: wheel odom: " << wheelDs->size() << ", GPS odom: " << gpsDs->size() << ".");
        return;
    }
    ROS_WARN_STREAM("No wheel odom data found around any GPS odom data. Clearing all data...");
    wheelDs->clear();
    gpsDs->clear();
}

void Matcher::gpggaCallback(const nmea_msgs::Gpgga::ConstPtr &msg)
{
    gpsQauality = msg->gps_qual;
}

void Matcher::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!poseMatchStarted)
    {
        return;
    }
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
    if (wheelDs->push(msg))
        return;
    ROS_DEBUG("Wheel odom buffer overflow, removing old data.");
    removeOldestDataBlock();
    assert(wheelDs->push(msg));
}

void Matcher::gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!poseMatchStarted || wheelDs->size() < 2)
    {
        return;
    }
    if (gpsQauality != 4 && !(useFloatingRTK && gpsQauality == 5))
    {
        ROS_WARN("GPS quality insufficient, skipping data.");
        return;
    }
    int wheelDsId = wheelDs->size() - 1;
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
        ROS_DEBUG("GPS odom buffer overflow, clearing buffer.");
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
    findTransform();
}

bool Matcher::startMatchCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    gpsDs->clear();
    wheelDs->clear();
    poseMatchStarted = true;
    res.success = true;
    res.message = "Pose match started.";
    return true;
}

bool Matcher::stopMatchCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    poseMatchStarted = false;
    res.success = true;
    res.message = "Pose match stopped.";
    return true;
}

bool Matcher::sendPoseEstCallback(SetPose::Request &req, SetPose::Response &res)
{
    const std::lock_guard<std::mutex> lock(mutex);
    
    tf2::Quaternion q(req.pose.pose.pose.orientation.x, req.pose.pose.pose.orientation.y, req.pose.pose.pose.orientation.z, req.pose.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    R.val[0][0] = m[0][0]; R.val[0][1] = m[0][1];
    R.val[1][0] = m[1][0]; R.val[1][1] = m[1][1];

    T.val[0][0] = req.pose.pose.pose.position.x;
    T.val[1][0] = req.pose.pose.pose.position.y;
    
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.translation.x = T.val[0][0];
    transformStamped.transform.translation.y = T.val[1][0];
    transformStamped.header.stamp = ros::Time::now();

    isTransformReady = true;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformStamped);

    mutex.unlock();

    return true;
}

bool Matcher::flipMatchCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    flipMatch = -1 * flipMatch;

    return true;
}

void Matcher::periodicUpdate(const ros::TimerEvent& event)
{
    static tf2_ros::TransformBroadcaster br;
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
}

double Matcher::getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}