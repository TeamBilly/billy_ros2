#ifndef BILLY_DESCRIPTION__BILLY_DESCRIPTION_HPP_
#define BILLY_DESCRIPTION__BILLY_DESCRIPTION_HPP_

#include "billy_description/visibility_control.h"

namespace billy_description
{

class BillyDescription
{
public:
    BillyDescription();

    virtual ~BillyDescription();

    enum mode{idle, teleop, half_turn, straight};

    struct motion{
        const float LIN_X_MAX = 2.0; // in m/s
        const float ANG_Z_MAX = 3.14; // in rad/s
        const float ACCEL_MAX = 1.0;
        const float DECEL_MAX = -1.0;
    };

    motion MOTION;
    
};

}  // namespace billy_description

#endif  // BILLY_DESCRIPTION__BILLY_DESCRIPTION_HPP_
