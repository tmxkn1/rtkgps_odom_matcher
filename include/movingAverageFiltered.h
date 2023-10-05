#pragma once

#include <ros/ros.h>
#include <deque>

namespace rtkgps_odom_matcher
{
class MovingAverageFiltered
{
public:
    MovingAverageFiltered(uint32_t size, double outlierThreshold);
    ~MovingAverageFiltered();
    void update(double newval);
    double getAverage();
    void reset();

private:
    uint32_t size;
    double outlierThreshold;
    std::deque<double> *buffer;
    uint32_t count;
    double sum;
    uint32_t index;
};
}