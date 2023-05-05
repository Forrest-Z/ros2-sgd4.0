#ifndef SGD_COSTMAP_PLUGINS__CIRCLE_RASTERIZER_HPP_
#define SGD_COSTMAP_PLUGINS__CIRCLE_RASTERIZER_HPP_

#include <vector>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

class CircleRasterizer
{
private:
    const int radius_;

    void setOutline(int x, int y)
    {
        // set 8 pixels
        maxs[radius_ + x] = y + radius_;
        maxs[radius_ + y] = x + radius_;
        maxs[radius_ - y] = x + radius_;

        mins[radius_ + x] = y - radius_;
        mins[radius_ + y] = x - radius_;
        mins[radius_ - y] = x - radius_;

        if (x != 0)
        {
            maxs[radius_ - x] = y + radius_;
            mins[radius_ - x] = y - radius_;
        }
    }
public:
    CircleRasterizer(int radius) : radius_(radius < 1 ? 1 : radius)
    {
        PLOGI << "Init CircleRasterizer with radius " << radius_;
        // initialize mins and maxs to size 2*radius 
        mins.resize(2*radius_+1);
        maxs.resize(2*radius_+1);

        // set initial values (origin is at 0,0)
        int x_ = 0;
        int y_ = radius_;
        // save initial pixel
        setOutline(x_, y_);

        // decision parameter
        int dec_param = 3 - 2 * radius_;

        while (y_ >= x_)
        {
            x_++;
            if (dec_param > 0)
            {
                y_--;
                dec_param = dec_param + 4 * (x_ - y_) + 10;
            }
            else
            {
                dec_param = dec_param + 4 * x_ + 6;
            }
            // save pixel
            setOutline(x_, y_);
        }
    }

    ~CircleRasterizer() {}

    std::vector<int> mins;
    std::vector<int> maxs;
};

#endif