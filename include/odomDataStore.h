#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>

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
    int seekTime(double time, double tolerance);

    private:
    std::deque<double> positionBuffer;
    std::deque<double> timeBuffer;
    int maxBufferSize;

    int validateIndex(int index);
};
}