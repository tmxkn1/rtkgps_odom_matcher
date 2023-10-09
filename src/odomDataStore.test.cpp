#include <gtest/gtest.h>
#include "odomDataStoreD.h"
#include <ros/ros.h>

using namespace rtkgps_odom_matcher;


TEST(OdomDataStoreTest, PushAndSize)
{
    int size = 5000;
    OdomDataStore store(size);
    EXPECT_EQ(store.size(), 0);

    nav_msgs::Odometry odom[size];
    for (int i = 0; i < size; i++)
    {
        odom[i].pose.pose.position.x = i;
        odom[i].pose.pose.position.y = i;
        odom[i].header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
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

TEST(OdomDataStoreTest, SeekTime)
{
    int size = 5000;
    OdomDataStore store(size);
    nav_msgs::Odometry odom[size];
    for (int i = 0; i < size; i++)
    {
        odom[i].pose.pose.position.x = i;
        odom[i].pose.pose.position.y = i;
        odom[i].header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
        EXPECT_TRUE(store.push(odom_));
    }

    for (int i = 0; i < size; i++)
    {
        EXPECT_EQ(store.seekTime(1.0*i, true, 2), i);
        EXPECT_EQ(store.seekTime(1.0*i, false, 2), i);
        EXPECT_EQ(store.seekTime(1.0*i+0.2, true, 2), i);
        EXPECT_EQ(store.seekTime(1.0*i+0.2, false, 2), i);
    }
    EXPECT_EQ(store.seekTime(1.3, true, .1), -1); // outside tolerance
    EXPECT_EQ(store.seekTime(1.3, false, .1), -1);
    EXPECT_EQ(store.seekTime(size, true, 2), size-1); // outside range
    EXPECT_EQ(store.seekTime(size, false, 2), size-1);
    EXPECT_EQ(store.seekTime(size+5, true, 2), -1); // outside tolerance & range
    EXPECT_EQ(store.seekTime(size+5, false, 2), -1);
}

TEST(OdomDataStoreTest, GetPosition)
{
    int size = 5;
    OdomDataStore store(size);

    nav_msgs::Odometry odom[size];
    for (int i = 0; i < size; i++)
    {
        odom[i].pose.pose.position.x = 2*i+1;
        odom[i].pose.pose.position.y = 2*i+2;
        odom[i].header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
        store.push(odom_);
    }
    double position[2];
    for (int i = 0; i < size; i++)
    {
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], 2*i+1);
        EXPECT_DOUBLE_EQ(position[1], 2*i+2);
    }
    EXPECT_FALSE(store.getPosition(size, position)); // overflow

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
}

TEST(OdomDataStoreTest, DiscardFront)
{
    int size = 10;
    int shift = 4;
    double tol = 2;
    OdomDataStore store(size);

    nav_msgs::Odometry odom[size];
    for (int i = 0; i < size; i++)
    {
        odom[i].pose.pose.position.x = 2*i+1;
        odom[i].pose.pose.position.y = 2*i+2;
        odom[i].header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
        store.push(odom_);
    }

    store.discardFront(shift);
    EXPECT_EQ(store.size(), size-shift);
    double position[2];
    for (int i = 0; i < size-shift; i++)
    {
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], 2*(i+shift)+1);
        EXPECT_DOUBLE_EQ(position[1], 2*(i+shift)+2);
    }

    double time[] = { 0,  1,  2,  3, 4, 5, 6, 7, 8, 9};
    int ind[]     = {-1, -1, -1, -1, 0, 1, 2, 3, 4, 5};
    for (int i = 4; i < size; i++)
    {
        EXPECT_EQ(store.getTime(ind[i]), time[i]);
        EXPECT_EQ(store.seekTime(time[i], true, 2), ind[i]);
        EXPECT_EQ(store.seekTime(time[i], false, 2), ind[i]);
    }
    EXPECT_EQ(store.getTime(-1), -1);
    EXPECT_EQ(store.getTime(6), -1);

    EXPECT_EQ(store.seekTime(0, true, 2), -1);
    EXPECT_EQ(store.seekTime(0, false, 2), -1);
    EXPECT_EQ(store.seekTime(1, true, 2), -1);
    EXPECT_EQ(store.seekTime(1, false, 2), -1);
    EXPECT_EQ(store.seekTime(2, true, 2), 0);
    EXPECT_EQ(store.seekTime(2, false, 2), 0);
    EXPECT_EQ(store.seekTime(11, true, 2), 5);
    EXPECT_EQ(store.seekTime(11, false, 2), 5);
    EXPECT_EQ(store.seekTime(13, true, 2), -1);
    EXPECT_EQ(store.seekTime(13, false, 2), -1);
}

TEST(OdomDataStoreTest, DiscardFrontThenPush)
{
    int size = 10;
    int shift = 4;
    double tol = 2;
    OdomDataStore store(size);

    nav_msgs::Odometry odom[size];
    for (int i = 0; i < size; i++)
    {
        odom[i].pose.pose.position.x = 2*i+1;
        odom[i].pose.pose.position.y = 2*i+2;
        odom[i].header.stamp = ros::Time(1.0*i);
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
        store.push(odom_);
    }

    store.discardFront(shift);
    for (int i = 0; i < shift; i++)
    {
        odom[i].pose.pose.position.x = 2*(i+size)+1;
        odom[i].pose.pose.position.y = 2*(i+size)+2;
        odom[i].header.stamp = ros::Time(1.0*(i+size));
        nav_msgs::Odometry::ConstPtr odom_(new nav_msgs::Odometry(odom[i]));
        EXPECT_TRUE(store.push(odom_));
    }
    
    double position[2];
    for (int i = 0; i < size; i++)
    {
        EXPECT_TRUE(store.getPosition(i, position));
        EXPECT_DOUBLE_EQ(position[0], 2*(i+shift)+1);
        EXPECT_DOUBLE_EQ(position[1], 2*(i+shift)+2);
    }

    double time[] = {10, 11, 12, 13, 4, 5, 6, 7, 8, 9};
    int ind[]     = { 6,  7,  8,  9, 0, 1, 2, 3, 4, 5};
    for (int i = 0; i < size; i++)
    {
        EXPECT_EQ(store.getTime(ind[i]), time[i]);
        EXPECT_EQ(store.seekTime(time[i], true, 2), ind[i]);
        EXPECT_EQ(store.seekTime(time[i], false, 2), ind[i]);
    }
    EXPECT_EQ(store.getTime(11), -1);
    EXPECT_EQ(store.getTime(11), -1);

    EXPECT_EQ(store.seekTime(0, true, 2), -1);
    EXPECT_EQ(store.seekTime(0, false, 2), -1);
    EXPECT_EQ(store.seekTime(1, true, 2), -1);
    EXPECT_EQ(store.seekTime(1, false, 2), -1);
    EXPECT_EQ(store.seekTime(2, true, 2), 0);
    EXPECT_EQ(store.seekTime(2, false, 2), 0);
    EXPECT_EQ(store.seekTime(4, true, 2), 0);
    EXPECT_EQ(store.seekTime(4, false, 2), 0);
    EXPECT_EQ(store.seekTime(14, true, 2), 9);
    EXPECT_EQ(store.seekTime(14, false, 2), 9);
    EXPECT_EQ(store.seekTime(16, true, 2), -1);
    EXPECT_EQ(store.seekTime(16, false, 2), -1);


    double position_full[size*2];
    EXPECT_TRUE(store.getPosition(position_full));
    for (int i = 0; i < size; i++)
    {
        EXPECT_DOUBLE_EQ(position_full[2*i], 2*(i+shift)+1);
        EXPECT_DOUBLE_EQ(position_full[2*i+1], 2*(i+shift)+2);
    }

    double position_back[2];
    for (int i = 0; i < size; i++)
    {
        EXPECT_TRUE(store.getPositionBack(i, position_back));
        EXPECT_DOUBLE_EQ(position_back[0], 2*(size-1-i+shift)+1);
        EXPECT_DOUBLE_EQ(position_back[1], 2*(size-1-i+shift)+2);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "odomDataStoreTest");
    ros::NodeHandle nh;

    int re;
    ros::Time start = ros::Time::now();
    for (int i = 0; i < 20; i++) {
        re = RUN_ALL_TESTS();
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Test time: " << (end-start).toSec()/20 << " s");
    return re;
}