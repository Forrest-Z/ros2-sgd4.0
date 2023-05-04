#include "sgd_costmap_plugins/triangle_rasterizer.hpp"

namespace sgd_costmap_plugins
{

TriangleRasterizer::TriangleRasterizer(int x0, int y0, int x1, int y1, int x2, int y2)
{
    int points[3][2] = {{x0,y0}, {x1,y1}, {x2,y2}};

    // get leftmost point -> smallest x value
    int lmp = 0;    // index of leftmost point
    int rmp = 0;    // index of rightmost point
    for (int i = 0; i < 3; i++)
    {
        if (points[i][0] < points[lmp][0])
        {
            lmp = i;
        }
        else if (points[i][0] > points[rmp][0])
        {
            rmp = i;
        }
    }

    // initialize triangle points
    x_lmp_ = points[lmp][0];
    y_lmp_ = points[lmp][1];
    x1_ = points[3-(lmp+rmp)][0];
    y1_ = points[3-(lmp+rmp)][1];
    x_rmp_ = points[rmp][0];
    y_rmp_ = points[rmp][1];

    // initialize dx & dy
    // int dx1 = abs(x1_ - x_lmp_);
    // int dy1 = abs(y1_ - y_lmp_);
    int dx2 = abs(x_rmp_ - x_lmp_);
    int dy2 = abs(y_rmp_ - y_lmp_);

    mins.resize(dx2+1);
    maxs.resize(dx2+1);
    for (int i = 0; i < mins.size(); i++)
    {
        mins[i] = -1;
        maxs[i] = -1;
    }

    plotPixel(points[lmp][0], points[lmp][1], points[rmp][0], points[rmp][1]);
    plotPixel(points[lmp][0], points[lmp][1], points[3-(lmp+rmp)][0], points[3-(lmp+rmp)][1]);
    plotPixel(points[3-(lmp+rmp)][0], points[3-(lmp+rmp)][1], points[rmp][0], points[rmp][1]);
}

TriangleRasterizer::~TriangleRasterizer() {}

void
TriangleRasterizer::plotPixel(int x1, int y1, int x2, int y2)
{
    //PLOGI.printf("Plot pixel: (%d, %d), (%d, %d)", x1, y1, x2, y2);
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    // wenn dx > dy dann normal (changeXY = false), sonst tausche x und y
    bool changeXY = dx <= dy;
    bool isxplus = x1 <= x2;
    int pk = changeXY ? 2 * dx - dy : 2 * dy - dx;

    for (int i = 0; i <= dx; i++) {
        setOutlinePnt(x1, y1);

        x1++;

        if (pk < 0 && changeXY) {
            int index = 0;
            while(pk < 0 && y1 != y2 && index < 100)
            {
                index++;
                pk = pk + 2 * dx;
                y1 < y2 ? y1++ : y1--;
            }
            pk = pk + 2 * dx - 2 * dy;

            setOutlinePnt(x1-1, y1);
            y1 < y2 ? y1++ : y1--;
        }
        else if (pk < 0)
        {
            pk = pk + 2 * dy;
        }
        else {
            y1 < y2 ? y1++ : y1--;
            pk = changeXY ? pk + 2 * dx - 2 * dy : pk + 2 * dy - 2 * dx;
        }
    }
    setOutlinePnt(x2, y2);
}

void
TriangleRasterizer::setOutlinePnt(int x, int y)
{
    if (x-x_lmp_ >= mins.size())
    {
        PLOGI.printf("x-x_lmp_ is larger than mins.size(): x-x_lmp_ = %d, mins.size() = %d",
                x-x_lmp_, mins.size());
    }
    if (x-x_lmp_ < 0)
    {
        PLOGI.printf("x-x_lmp_ is smaller than 0: x-x_lmp_ = %d, mins.size() = %d",
                x-x_lmp_, mins.size());
    }
    if (mins[x-x_lmp_] < 0 || mins[x-x_lmp_] > y)
    {
        mins[x-x_lmp_] = y;
    }
    if (maxs[x-x_lmp_] < y)
    {
        maxs[x-x_lmp_] = y;
    }
}

}