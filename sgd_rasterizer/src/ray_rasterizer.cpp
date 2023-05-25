#include "sgd_rasterizer/ray_rasterizer.hpp"

namespace sgd_rasterizer
{

RayRasterizer::RayRasterizer(int width, int height)
    : width_(width), height_(height)
{
    data.resize(width * height);

    // init logging
    PLOGD << "Initialized RayRasterizer object";
}

RayRasterizer::~RayRasterizer() {}

void
RayRasterizer::add_object(std::vector<float> vertices, uint8_t confidence)
{
    PLOGV.printf("add_object(vertices.size() = %d, %u)", vertices.size(), confidence);
    // get bottom and leftmost point
    // get rightmost point
    x_lmp = vertices[0];
    y_bottom = vertices[1];
    x_rmp = vertices[0];
    y_top = vertices[1];

    for (int i = 0; i < vertices.size(); i+=2)
    {
        // i is x value, i+1 is y value
        if (vertices[i] > x_rmp)    x_rmp = vertices[i];
        if (vertices[i] < x_lmp)    x_lmp = vertices[i];
        if (vertices[i+1] < y_bottom)   y_bottom = vertices[i+1];
        if (vertices[i+1] > y_top)      y_top = vertices[i+1];
    }

    PLOGV.printf("Object has bounds (right/left/bottom/top): %.2f, %.2f, %.2f, %.2f", x_rmp, x_lmp, y_bottom, y_top);

    // ray from x0, y0 -> x_max, y0
    // get last point from polygon list
    float x0_ = vertices[vertices.size()-2];
    float y0_ = vertices[vertices.size()-1];
    float x1_, y1_;

    //int obj_height_ = (int)round(y_top-y_bottom);
    int x_lmpi = (int)round(x_lmp);

    int y_start = ceil(y_bottom);
    int y_end = floor(y_top);
    std::forward_list<int> x_list_new;
    PLOGV.printf("Find intersections from %d to %d", y_start, y_end);
    for (int yi = y_start; yi <= y_end; yi++)
    {
        if (yi < 0 || yi >= height_)     continue;
        for (int j = 0; j < vertices.size()-1; j+=2)
        {
            x1_ = vertices[j];
            y1_ = vertices[j+1];

            if ((y0_ > yi && y1_ > yi) || (y0_ < yi && y1_ < yi))
            {
                x0_ = x1_;
                y0_ = y1_;
                continue;
            }

            // TODO single point detection
            int xi = get_intersection(x0_, y0_, x1_, y1_, yi);
            // if (xi < 0)
            // {
            //     x0_ = x1_;
            //     y0_ = y1_;
            //     continue;
            // }
            // add point to set
            PLOGV.printf("Line (%.2f, %.2f) -> (%.2f, %.2f) ==> Intersection (%d, %d)",
                    x0_, y0_, x1_, y1_, xi, yi);
            x_list_new.push_front(xi);      // xi-x_lmpi
            x0_ = x1_;
            y0_ = y1_;
        }

        // paint line
        // int cell_start = yi * width_ + x_lmpi;
        // int x_cell = x_lmpi < 1 ? 1 : x_lmpi;
        uint8_t what = 0;
        x_list_new.sort();

        int x_cell = *x_list_new.begin();
        for (auto item : x_list_new)
        {
            // do nothing up to first point
            if (what == 0)
            {
                x_cell = item;
                what = confidence;
                continue;
            }
            
            fill_line(yi, x_cell, item, what);
            what = 0;
        }
        
        x_list_new.clear();
    }
}

void
RayRasterizer::reset()
{
    PLOGV << "reset()";
    data.clear();
    data.resize(width_ * height_);
}

int
RayRasterizer::get_intersection(float x0, float y0, float x1, float y1, float py)
{
    PLOGV.printf("get_intersection(%.2f, %.2f, %.2f, %.2f, %.2f)", x0, y0, x1, y1, py);
    // calculate intersection point -> only x is required, y is given
    // line 1: (x_lmp, py) -> (x_rmp, py) -> p1/p2
    // line 2: (x0, y0) -> (x1, y1) -> p3/p4
    float u = (x_lmp*(y0-py) + x_rmp*(py-y0))
            / (x_lmp*(y0-y1) + x_rmp*(y1-y0));
    return (int)round(x0+u*(x1-x0));   // intersection point
}

void
RayRasterizer::fill_line(int y, int from, int to, uint8_t what)
{
    from = std::max(from, 0);
    to = std::min(width_-1, to);
    PLOGV.printf("fill_line(%d, %d, %d, %u)", y, from, to, what);

    try
    {
        for (int i = y*width_ + from; i <= y*width_+to; i++)
        {
            data.at(i) = std::max(what, data.at(i));
        }
    }
    catch(const std::out_of_range& e)
    {
        PLOGE.printf("RayRasterizer encountered an error: %s", e.what());
        PLOGE.printf("y = %d, from = %d, to = %d", y, from, to);
    }   
}

} // namespace sgd_rasterizer