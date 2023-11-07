#ifndef __COMETS_MATH_H__
#define __COMETS_MATH_H__

#include <cmath>

namespace comets
{
    inline constexpr bool in_range(double value, double low, double high)
    {
        return value >= low && value <= high;
    }

    inline constexpr bool approx_equal(double a, double b, double epsilon) {
        return std::abs(a - b) < epsilon;
    }
} // namespace comets

#endif
