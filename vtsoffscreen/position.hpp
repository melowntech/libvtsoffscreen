/**
 * Copyright (c) 2018 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
