#ifndef SGD_HARDWARE__TOUCH_FILTER_HPP_
#define SGD_HARDWARE__TOUCH_FILTER_HPP_

#include <list>
#include <math.h>

namespace sgd_hardware_drivers
{

class MovingAverageFilter
{
private:
    uint size_;
    std::list<int> data;
public:
    MovingAverageFilter();
    ~MovingAverageFilter();

    void set_filter_size(int size);
    void add_value(int data);
    double mov_avg();
};

}

#endif