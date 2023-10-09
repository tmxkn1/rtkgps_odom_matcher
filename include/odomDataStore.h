#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace rtkgps_odom_matcher
{

class OdomDataStore
{
    public:
    OdomDataStore();
    OdomDataStore(int maxBufferSize);
    ~OdomDataStore();

    bool push(nav_msgs::Odometry::ConstPtr odom);
    void clear();
    int size();
    bool discardFront(int index);
    bool getPosition(double* position);
    bool getPosition(int index, double* position);
    bool getPositionBack(int index, double* position);
    double getTime(int index);
    double getTimeBack(int index);
    int seekTime(double time, bool forward, double tolerance);

    private:
    double* positionBuffer;
    double* timeBuffer;
    int pointer;
    int maxBufferSize;
    int front;
    int count;

    int validateIndex(int index);
};
}