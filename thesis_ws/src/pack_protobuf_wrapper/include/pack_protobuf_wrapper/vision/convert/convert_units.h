#ifndef _CONVERT_UNIT
#define _CONVERT_UNIT

#include <cmath>

namespace pr {

/**
 * Converts the milimeters used by SSL vision to meters.
 * millimeters -> meters
 */
float mm_to_m(float scalar);
}

#endif // _CONVERT_UNIT