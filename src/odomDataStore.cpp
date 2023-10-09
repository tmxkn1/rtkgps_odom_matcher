#include "odomDataStore.h"

using namespace rtkgps_odom_matcher;

OdomDataStore::OdomDataStore()
{}

OdomDataStore::OdomDataStore(int maxBufferSize)
{
    this->maxBufferSize = maxBufferSize;
    positionBuffer = (double*)calloc(maxBufferSize*2, sizeof(double));
    timeBuffer = (double*)calloc(maxBufferSize, sizeof(double));
    clear();
}

OdomDataStore::~OdomDataStore()
{
    std::free(positionBuffer);
    std::free(timeBuffer);
}

bool OdomDataStore::push(nav_msgs::Odometry::ConstPtr odom){
    if (count == maxBufferSize)
    {
        ROS_WARN("OdomDataStore buffer full. Data disgarded.");
        return false;
    }

    pointer++;
    if (pointer >= maxBufferSize)
    {
        pointer = 0;
        ROS_WARN("OdomDataStore buffer overflow. Shifting data left.");
    }
    // ROS_INFO_STREAM("Current pointer: " << pointer << ", maxBufferSize: " << maxBufferSize << ", count: " << count << ".");
    positionBuffer[pointer*2] = odom->pose.pose.position.x;
    positionBuffer[pointer*2+1] = odom->pose.pose.position.y;
    timeBuffer[pointer] = odom->header.stamp.toSec();
    count ++;
    return true;
}

void OdomDataStore::clear()
{
    pointer = -1;
    front = 0;
    count = 0;
}

int OdomDataStore::size()
{
    return count;
}

bool OdomDataStore::discardFront(int index)
{
    index = validateIndex(index);
    if (index < 0)
        return false;
    front = index;
    count -= index;
    assert(count >= 0);
    return true;
}

bool OdomDataStore::getPosition(double* position)
{
    if (pointer < 0)
        return false;
    if (pointer > front)
        memcpy(position, &positionBuffer[front*2], 2*count*sizeof(double));
    else
    {
        memcpy(position, &positionBuffer[front*2], 2*(maxBufferSize-front)*sizeof(double));
        memcpy(position+2*(maxBufferSize-front), positionBuffer, 2*(pointer+1)*sizeof(double));
    }
    return true;
}

bool OdomDataStore::getPosition(int index, double* position)
{
    index = validateIndex(index);
    if (index < 0)
        return false;
    memcpy(position, &positionBuffer[index*2], 2*sizeof(double));
    return true;
}

bool OdomDataStore::getPositionBack(int index, double* position)
{
    return getPosition(count-index-1, position);
}

double OdomDataStore::getTime(int index)
{
    index = validateIndex(index);
    if (index < 0)
        return -1;
    return timeBuffer[index];
}

double OdomDataStore::getTimeBack(int index)
{
    return getTime(count-index);
}

int OdomDataStore::seekTime(double time, bool forward=true, double tolerance=0.5){ 
    if (pointer < 0)
    {
        return -1;
    }
    int k = 1;
    int direction = forward ? 1 : -1;
    int i = forward ? front : pointer;

    // initial state
    double minDiff = abs(timeBuffer[i] - time);

    while (k < count)
    {
        i += direction;
        if (i >= maxBufferSize)
        {
            i = 0;
        }
        else if (i < 0)
        {
            i = maxBufferSize - 1;
        }

        double diff = abs(timeBuffer[i] - time);
        if (diff < minDiff)
        {
            minDiff = diff;
        }
        else
        {
            i -= direction;
            if (i < 0)
            {
                i += maxBufferSize;
            }
            break;
        }
        k++;
    }
    if (minDiff > tolerance)
    {
        return -1;
    }
    i -= front;
    if (i < 0)
    {
        i += maxBufferSize;
    }
}

int OdomDataStore::validateIndex(int index)
{
    if (pointer < 0 || index < 0)
        return -1;
    index += front;
    if (index >= maxBufferSize)
    {
        if (front < pointer)
            return -1;
        index -= maxBufferSize;
        if (index > pointer)
            return -1;
    }
    return index;
}