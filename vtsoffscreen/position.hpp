#ifndef vtsoffscreen_position_hpp_included_
#define vtsoffscreen_position_hpp_included_

#include <string>

#include <boost/variant.hpp>

#ifdef VTSOFFSCREEN_HAS_OPTICS
#include "optics/camera.hpp"
#endif

namespace vts::offscreen {

/** VTS position in JSON format.
 */
struct VtsJsonPosition {
    std::string position;
};

/** VTS serialized position. Comma-separated list of 10 elements.
 */
struct VtsSerializedPosition {
    std::string position;
};

#ifdef VTSOFFSCREEN_HAS_OPTICS

/** Intrinsics and extrinsics defined in liboptics (only when available).
 */
struct OpticsPosition {
    /** Intrinsic parameters.
     */
    optics::Camera::Parameters camera;

    /** Extrinsic parameters.
     */
    optics::Camera::Position position;

    // NB: viewport is present in view itself.
};

#endif

typedef boost::variant<VtsSerializedPosition
#ifdef VTSOFFSCREEN_HAS_OPTICS
                       , OpticsPosition
#endif
                       > Position;

} // namespace vts::offscreen

#endif // vtsoffscreen_position_hpp_included_
