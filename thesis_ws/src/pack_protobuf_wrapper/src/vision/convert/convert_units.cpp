
#include "pack_protobuf_wrapper/vision/convert/convert_units.h"

namespace pr {
/**
 * Converts the milimeters used by SSL vision to meters.
 * millimeters -> meters
 */
float mm_to_m(float scalar) {
    return scalar / 1000;
}
}
