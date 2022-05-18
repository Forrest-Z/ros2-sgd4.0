#ifndef SGD_HARDWARE__TOUCH_FILTER_HPP_
#define SGD_HARDWARE__TOUCH_FILTER_HPP_

#include <list>
#include <math.h>

namespace sgd_util
{

class SensorFilter
{
private:
    uint size_;
    std::list<double> datalist_;
public:
    SensorFilter()
    {
        size_ = 10;
    }

    ~SensorFilter() {};

    /**
     * @brief Set the filter size object
     * 
     * @param size 
     */
    void set_filter_size(int size)
    {
        size_ = size;
    }

    void add_value(double data)
    {
        // add newest value to front of list
        datalist_.push_front(data);

        while (datalist_.size() > size_)
        {
            datalist_.pop_back();
        }
    }

    /**
     * @brief 
     * 
     * 
     * @return double 
     */
    double result()
    {
        // calculate average over list
        double avg = 0.0;
        for (double n : datalist_)
        {
            avg += n;
        }
        return avg / (double)size_;
    }
};

}

#endif