#include <gtest/gtest.h>
#include "odomDataStore.h"
#include <ros/ros.h>
#include <deque>

using namespace rtkgps_odom_matcher;

void buildDataStore(OdomDataStore &store, int size, std::deque<nav_msgs::Odometry> &odom)
{
    for (int i = 0; i < size; i++)
    {
        nav_msgs::Odometry _odom;
        _odom.pose.pose.position.x = 2*i+1;
        _odom.pose.pose.position.y = 2*i+2;
        _odom.header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr __odom(new nav_msgs::Odometry(_odom));
        odom.push_back(_odom);
        store.push(__odom);
    }
    assert(store.size() == size);
    assert(odom.size() == size);
}

void discardFront(OdomDataStore &store, int num)
{
    store.discardFront(num);
}

void pushNew(OdomDataStore &store, int num, std::deque<nav_msgs::Odometry> &odom)
{
    nav_msgs::Odometry odom_back = odom.back();
    int tStart = odom.back().header.stamp.toSec();
    int xStart = odom.back().pose.pose.position.x;
    int yStart = odom.back().pose.pose.position.y;
    for (int i = 0; i < num; i++)
    {
        nav_msgs::Odometry _odom;
        _odom.pose.pose.position.x = 2*i+1+xStart;
        _odom.pose.pose.position.y = 2*i+2+yStart;
        _odom.header.stamp = ros::Time(1.0*i+1+tStart);
        nav_msgs::Odometry::ConstPtr __odom(new nav_msgs::Odometry(_odom));
        odom.push_back(_odom);
        store.push(__odom);
    }
}

TEST(OdomDataStoreTest, PushAndSize)
{
    int size = 5000;
    OdomDataStore store(size);
    EXPECT_EQ(store.size(), 0);

    nav_msgs::Odometry odom;
    for (int i = 0; i < size; i++)
    {
        odom.pose.pose.position.x = i;
        odom.pose.pose.position.y = i;
        odom.header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom));
        EXPECT_TRUE(store.push(odom_));
        EXPECT_EQ(store.size(), i+1);
    }

    nav_msgs::Odometry odom_extra;
    odom_extra.pose.pose.position.x = 2*size+2;
    odom_extra.pose.pose.position.y = 2*size+3;
    odom_extra.header.stamp = ros::Time(2*size+2);
    nav_msgs::Odometry::ConstPtr odom_extra_(new nav_msgs::Odometry(odom_extra));
    EXPECT_FALSE(store.push(odom_extra_));
    EXPECT_EQ(store.size(), size);
}

TEST(OdomDataStoreTest, Clear)
{
    OdomDataStore store(1000);
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 2.0;
    odom.header.stamp = ros::Time(1.0);
    nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom));
    EXPECT_TRUE(store.push(odom_));
    EXPECT_EQ(store.size(), 1);
    store.clear();
    EXPECT_EQ(store.size(), 0);
}

TEST(OdomDataStoreTest, GetPositionFrontFullAndBack)
{
    int size = 5000;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    
    double position[2];
    for (int i = 0; i < size; i++)
    {
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], odom[i].pose.pose.position.x);
        EXPECT_DOUBLE_EQ(position[1], odom[i].pose.pose.position.y);
    }
    EXPECT_FALSE(store.getPosition(size, position));
    EXPECT_FALSE(store.getPosition(-1, position));

    double position_full[size*2];
    EXPECT_TRUE(store.getPosition(position_full));
    for (int i = 0; i < size; i++)
    {
        EXPECT_DOUBLE_EQ(position_full[2*i], 2*i+1);
        EXPECT_DOUBLE_EQ(position_full[2*i+1], 2*i+2);
    }

    double position_back[2];
    for (int i = 0; i < size; i++)
    {
        EXPECT_TRUE(store.getPositionBack(i, position_back));
        EXPECT_DOUBLE_EQ(position_back[0], 2*(size-1-i)+1);
        EXPECT_DOUBLE_EQ(position_back[1], 2*(size-1-i)+2);
    }
    EXPECT_FALSE(store.getPositionBack(size, position));
    EXPECT_FALSE(store.getPositionBack(-1, position));
}

TEST(OdomDataStoreTest, GetTimeFrontAndBack)
{
    int size = 5000;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    
    for (int i = 0; i < size; i++)
    {
        EXPECT_DOUBLE_EQ(store.getTime(i), odom[i].header.stamp.toSec());
        EXPECT_DOUBLE_EQ(store.getTimeBack(size-i-1), odom[i].header.stamp.toSec());
    }
    EXPECT_EQ(store.getTime(-1), -1);
    EXPECT_EQ(store.getTime(size+1), -1);
    EXPECT_EQ(store.getTimeBack(-1), -1);
    EXPECT_EQ(store.getTimeBack(size+1), -1);
}

TEST(OdomDataStoreTest, DiscardFront)
{
    int size = 5000;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    EXPECT_FALSE(store.discardFront(-1));
    EXPECT_FALSE(store.discardFront(size));

    EXPECT_EQ(store.size(), size);
    EXPECT_TRUE(store.discardFront(0));
    EXPECT_EQ(store.size(), size);
    EXPECT_TRUE(store.discardFront(1));
    EXPECT_EQ(store.size(), size-1);
    EXPECT_TRUE(store.discardFront(size-3));
    EXPECT_EQ(store.size(), 2);
}

TEST(OdomDataStoreTest, SeekTime)
{
    int size = 5000;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    double time;

    for (int i = 0; i < size; i++)
    {
        time = odom[i].header.stamp.toSec();
        EXPECT_EQ(store.seekTime(time, 2), i);
        EXPECT_EQ(store.seekTime(time+0.2, 2), i);
    }
    time = odom.front().header.stamp.toSec();
    EXPECT_EQ(store.seekTime(time+0.3, .1), -1); // outside tolerance
    time = odom.back().header.stamp.toSec();
    EXPECT_EQ(store.seekTime(time+1, 2), size-1); // outside range
    EXPECT_EQ(store.seekTime(time+5, 2), -1); // outside tolerance & range
}

TEST(OdomDataStoreTest, DiscardFrontThenGetPositionAndTime)
{

    int size = 5000;
    int discardSize = 50;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    discardFront(store, discardSize);
    EXPECT_EQ(store.size(), size-discardSize);

    double position[2];
    for (int i = 0; i < size-discardSize; i++)
    {
        nav_msgs::Odometry odom_ = odom[i+discardSize];
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], odom_.pose.pose.position.x);
        EXPECT_DOUBLE_EQ(position[1], odom_.pose.pose.position.y);
        EXPECT_DOUBLE_EQ(store.getTime(i), odom_.header.stamp.toSec());
    }
}

TEST(OdomDataStoreTest, DiscardFrontThenSeekTime)
{

    int size = 5000;
    int discardSize = 50;
    int tolerance = 2;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    discardFront(store, discardSize);
    EXPECT_EQ(store.size(), size-discardSize);
    
    double time;
    for (int i = 0; i < size-discardSize; i++)
    {
        time = odom[i+discardSize].header.stamp.toSec();
        EXPECT_EQ(store.seekTime(time, tolerance), i);
    }

    for (int i = 0; i < discardSize-tolerance; i++)
    {
        time = odom[i].header.stamp.toSec();
        EXPECT_EQ(store.seekTime(time, tolerance), -1);
    }

    for (int i = 1; i <= tolerance; i++)
    {
        time = odom[discardSize-tolerance-1].header.stamp.toSec()+i;
        EXPECT_EQ(store.seekTime(time, tolerance), 0);

        time = odom.back().header.stamp.toSec()+i;
        EXPECT_EQ(store.seekTime(time, tolerance), size-discardSize-1);
    }
}

TEST(OdomDataStoreTest, DiscardPushThenGetPosAndTime)
{
    int size = 5000;
    int discardSize = 50;
    int tolerance = 2;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    discardFront(store, discardSize);
    EXPECT_EQ(store.size(), size-discardSize);
    pushNew(store, discardSize, odom);
    EXPECT_EQ(store.size(), size);

    double position[2];
    for (int i = 0; i < size; i++)
    {
        nav_msgs::Odometry odom_ = odom[i+discardSize];
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], odom_.pose.pose.position.x);
        EXPECT_DOUBLE_EQ(position[1], odom_.pose.pose.position.y);
        EXPECT_DOUBLE_EQ(store.getTime(i), odom_.header.stamp.toSec());
    }
}

TEST(OdomDataStoreTest, DiscardPushThenSeekTime)
{
    int size = 10;
    int discardSize = 5;
    int tolerance = 2;
    OdomDataStore store(size);
    std::deque<nav_msgs::Odometry> odom;
    buildDataStore(store, size, odom);
    discardFront(store, discardSize);
    EXPECT_EQ(store.size(), size-discardSize);
    pushNew(store, discardSize, odom);
    
    double time;
    for (int i = 0; i < 5; i++)
    {
        time = odom[i+discardSize].header.stamp.toSec();
        EXPECT_EQ(store.seekTime(time, tolerance), i);
    }

    for (int i = 0; i < discardSize-tolerance; i++)
    {
        time = odom[i].header.stamp.toSec();
        EXPECT_EQ(store.seekTime(time, tolerance), -1);
    }

    for (int i = 1; i <= tolerance; i++)
    {
        time = odom[discardSize-tolerance-1].header.stamp.toSec()+i;
        EXPECT_EQ(store.seekTime(time, tolerance), 0);

        time = odom.back().header.stamp.toSec()+i;
        EXPECT_EQ(store.seekTime(time, tolerance), size-1);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "odomDataStoreTest");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}