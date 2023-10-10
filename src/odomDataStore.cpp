#include "odomDataStore.h"

using namespace rtkgps_odom_matcher;

OdomDataStore::OdomDataStore()
{}

OdomDataStore::OdomDataStore(int maxBufferSize)
{
    this->maxBufferSize = maxBufferSize;
    clear();
}

OdomDataStore::~OdomDataStore()
{
}

bool OdomDataStore::push(nav_msgs::Odometry::ConstPtr odom){
    if (timeBuffer.size() >= maxBufferSize)
    {
        ROS_DEBUG("OdomDataStore buffer full. Data disgarded.");
        return false;
    }
    positionBuffer.push_back(odom->pose.pose.position.x);
    positionBuffer.push_back(odom->pose.pose.position.y);
    timeBuffer.push_back(odom->header.stamp.toSec());
    return true;
}

void OdomDataStore::clear()
{
    positionBuffer = std::deque<double>();
    timeBuffer = std::deque<double>();
}

int OdomDataStore::size()
{
    return timeBuffer.size();
}

bool OdomDataStore::discardFront(int index)
{
    if (validateIndex(index))
        return false;
    if (index == 0)
        return true;
    positionBuffer.erase(positionBuffer.begin(), positionBuffer.begin()+index*2);
    timeBuffer.erase(timeBuffer.begin(), timeBuffer.begin()+index);
    return true;
}

bool OdomDataStore::getPosition(double* position)
{
    std::copy(positionBuffer.begin(), positionBuffer.end(), position);
    return true;
}

bool OdomDataStore::getPosition(int index, double* position)
{
    if (validateIndex(index))
        return false;
    std::copy(positionBuffer.begin()+index*2, positionBuffer.begin()+index*2+2, position);
    return true;
}

bool OdomDataStore::getPositionBack(int index, double* position)
{
    if (validateIndex(index))
        return false;
    std::copy(positionBuffer.end()-index*2-2, positionBuffer.end()-index*2, position);
    return true;
}

double OdomDataStore::getTime(int index)
{   
    if (validateIndex(index))
        return -1;
    return timeBuffer[index];
}

double OdomDataStore::getTimeBack(int index)
{
    return getTime(timeBuffer.size()-index-1);
}

int OdomDataStore::seekTime(double time, double tolerance=0.5){ 
    int size = timeBuffer.size();
    if (size == 0)
        return -1;
        
    int i = 0;
    // initial state
    double minDiff = abs(timeBuffer[i] - time);
    for (double t: timeBuffer)
    {
        double diff = abs(t - time);
        if (diff <= minDiff)
        {
            minDiff = diff;
            i++;
            continue;
        }
        break;
    }
    if (minDiff > tolerance)
    {
        return -1;
    }
    return i-1;
}

int OdomDataStore::validateIndex(int index)
{
    return index < 0 || index >= timeBuffer.size();
}