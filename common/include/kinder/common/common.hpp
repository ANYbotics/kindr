/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>



// forward declarations

namespace kinder {
namespace quaternions {
namespace eigen_implementation {

template<typename PrimType>
class UnitQuaternion;

} // namespace eigen_implementation
} // namespace quaternions

namespace rotations {

enum class RotationUsage {
	ACTIVE,
	PASSIVE
};

namespace eigen_implementation {

template<typename PrimType, enum RotationUsage Usage>
class RotationQuaternion;

template<typename PrimType, enum RotationUsage Usage>
class AngularVelocity;

} // namespace eigen_implementation
} // namespace rotations
} // namespace rm



namespace kinder {
namespace common {


template<typename T>
T mod(const T& x, const T& y)
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
inline T wrapAngle(const T& angle, const T& x1, const T& x2)
{
    return Mod(angle-x1, x2-x1) + x1;
}

// wrap angle to [-PI..PI)
template<typename T>
inline T wrapPosNegPI(const T& angle)
{
    return Mod(angle + M_PI, 2*M_PI) - M_PI;
}

// wrap angle to [0..2*PI)
template<typename T>
inline T wrapTwoPI(const T& angle)
{
    return Mod(angle, 2*M_PI);
}

}
}


#endif /* COMMON_HPP_ */
