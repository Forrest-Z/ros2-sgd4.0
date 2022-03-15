#include "cap_touch/moving_average_filter.hpp"

namespace sgd_hardware_drivers
{

MovingAverageFilter::MovingAverageFilter()
{
    size_ = 5;
}

MovingAverageFilter::~MovingAverageFilter()
{
}

void
MovingAverageFilter::set_filter_size(int size)
{
    size_ = size;
}

void
MovingAverageFilter::add_value(int value)
{
    // add newest value to front of list
    data.push_front(value);

    while (data.size() > size_)
    {
        data.pop_back();
    }
}

double
MovingAverageFilter::mov_avg()
{
    // calculate average over list
    int avg = 0;
    for (int n : data)
    {
        avg += n;
    }
    return round(avg / size_);
}

}