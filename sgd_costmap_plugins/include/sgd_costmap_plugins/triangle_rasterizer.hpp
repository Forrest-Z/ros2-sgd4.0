#ifndef SGD_COSTMAP_PLUGINS__TRIANGLE_RASTERIZER_HPP_
#define SGD_COSTMAP_PLUGINS__TRIANGLE_RASTERIZER_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_costmap_plugins
{

class TriangleRasterizer
{
private:
    // lmp = leftmost point, rmp = rightmost point
    int x_lmp_;
    int y_lmp_;
    int x1_;
    int y1_;
    int x_rmp_;
    int y_rmp_;

    /**
     * @brief Set the Outline Pnt object
     * 
     * @param x 
     * @param y 
     */
    void setOutlinePnt(int x, int y);

    void plotPixel(int x1, int y1, int x2, int y2);
public:
    TriangleRasterizer(int x0, int y0, int x1, int y1, int x2, int y2);
    ~TriangleRasterizer();

    std::vector<int> mins;
    std::vector<int> maxs;

    /**
     * @brief Get the x coordinate of the leftmost point in global coordinates * resolution
     * 
     * @return int 
     */
    inline int get_lmp()
    {
        return x_lmp_;
    }
};


} // namespace sgd_costmap_plugins

#endif