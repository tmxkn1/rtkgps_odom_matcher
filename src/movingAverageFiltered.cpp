#include "movingAverageFiltered.h"

using namespace rtkgps_odom_matcher;

MovingAverageFiltered::MovingAverageFiltered(uint32_t size, double outlierThreshold)
: size(size), outlierThreshold(outlierThreshold), count(0), sum(0), index(0)
{
    buffer = new double[size];
}

MovingAverageFiltered::~MovingAverageFiltered()
{
    delete[] buffer;
}

void MovingAverageFiltered::update(double newval)
{
    if (count < size)
    {
        sum += newval;
        buffer[count] = newval;
        count++;
    }
    sum -= buffer[index];
    buffer[index] = newval;
    sum += newval;
    index = (index + 1) % size;
}

double MovingAverageFiltered::getAverage()
{
    return sum / count;
}

void MovingAverageFiltered::reset()
{
    count = 0;
    sum = 0;
    index = 0;
}

