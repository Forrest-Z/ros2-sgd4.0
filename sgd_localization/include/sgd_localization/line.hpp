#ifndef SGD_LOCALIZATION_LINE_HPP_
#define SGD_LOCALIZATION_LINE_HPP_

#include <string>
#include <math.h>
#include "point.hpp"

namespace sgd_localization
{

class Line
{
private:
    
public:
    const Point p1_;
    const Point p2_;

    /**
     * @brief Create new line
    */
    Line(Point p1, Point p2);
    ~Line() = default;

    /**
     * 
    */
    float length();

    /**
     * @brief returns the smallest angle between the two lines
    */
    float angle(Line other);

    inline Line operator=(const Line& ln)  { return Line(ln.p1_, ln.p2_); };
    // inline bool operator==(const Line& ln) { return p1_.x == ln.p1_.x; }
    // inline bool operator!=(const Line& ln) { return p1_.x != ln.p1_.x; }
};

} // namespace sgd_localization

#endif