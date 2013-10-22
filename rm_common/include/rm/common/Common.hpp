/*!
* @file     Common.hpp
* @author   Remo Diethelm
* @date     22 10, 2013
* @version  1.0
* @ingroup  rm
* @brief
*/

#ifndef COMMON_HPP_
#define COMMON_HPP_


#include <cmath>

#include <Eigen/Core>
//#include <Eigen/Geometry>

namespace rm {
namespace common {

// todo: static?


template<typename T>
T Mod(const T x, const T y)
{
    static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (y == 0.0)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , 2*M_PI ): m= -1.421e-14
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -2*M_PI): m= 1.421e-14
        }
    }

    return m;
}

// wrap angle to [x1..x2)
template<typename T>
inline T wrapAngle(const T angle, const T x1, const T x2)
{
    return Mod(angle-x1, x2-x1) + x1;
}

// wrap angle to [x1..x2)
template<typename T>
inline Eigen::AngleAxis<T> wrapAngle(const Eigen::AngleAxis<T> aa, const T x1, const T x2)
{
    return Eigen::AngleAxis<T>(wrapAngle(aa.angle(),x1,x2), aa.axis());
}

// wrap angle to [-PI..PI)
template<typename T>
inline T wrapPosNegPI(const T angle)
{
    return Mod(angle + M_PI, 2*M_PI) - M_PI;
}

// wrap angle to [0..2*PI)
template<typename T>
inline T wrapTwoPI(const T angle)
{
    return Mod(angle, 2*M_PI);
}




} // end namespace common
} // end namespace rm



#endif /* COMMON_HPP_ */
