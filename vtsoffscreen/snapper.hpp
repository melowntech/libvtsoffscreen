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

#ifndef vtsoffscreen_snapper_hpp_included_
#define vtsoffscreen_snapper_hpp_included_

#include <atomic>
#include <future>
#include <mutex>
#include <thread>
#include <queue>
#include <memory>

#include <opencv2/core/core.hpp>

#include <vts-browser/mapOptions.hpp>
#include <vts-browser/cameraOptions.hpp>

#include "geo/srsdef.hpp"
#include "glsupport/eglfwd.hpp"

#include "./position.hpp"

namespace vts { namespace offscreen {

using Image = cv::Mat_<cv::Vec3b>;

struct SnapperConfig {
    std::string mapConfigUrl;
    std::string authUrl;
    std::string mapView;

    /** Custom SRS #1, passed to created VTS map.
     */
    geo::SrsDefinition customSrs1;

    /** Custom SRS #2, passed to created VTS map.
     */
    geo::SrsDefinition customSrs2;

    MapCreateOptions confMapCreate;
    MapRuntimeOptions confMapRuntime;
    CameraOptions confCamera;

    uint32 antialiasingSamples; // two or more to enable multisampling
    bool renderAtmosphere;

    SnapperConfig();
};

/** View definition.
 */
struct View {
    /** View position. Choose from alternatives (see position.hpp for details).
     */
    Position position;

    /** Viewport.
     */
    math::Viewport2 viewport;

    /** Keypoints to sample in the scene.
     */
    math::Points2 keypoints;

    View() {}
};

struct Point {
    math::Point2 image;
    math::Point3 world;

    typedef std::vector<Point> list;

    Point(const math::Point2 &image, const math::Point3 &world)
        : image(image), world(world)
    {}
};

/** Photographed snapshot.
 */
struct Snapshot {
    /** Take photograph.
     */
    Image image;

    /** Samples keypoints.
     */
    Point::list keypoints;

    Snapshot(const math::Size2 &size);
};

class Snapper : public boost::noncopyable {
public:
    /** Run snapper on default native display.
     */
    Snapper(const SnapperConfig &config);

    /** Run snapper on provided EGL device.
     */
    Snapper(const SnapperConfig &config, const glsupport::egl::Device &device);

    ~Snapper();

    Snapshot snap(const View &view);

    struct Detail;

private:
    std::unique_ptr<Detail> detail_;
};

/** Asynchronous version.
 */
class AsyncSnapper : public boost::noncopyable {
public:
    AsyncSnapper(const SnapperConfig &config);

    ~AsyncSnapper();

    Snapshot operator()(const View &view);

private:
    struct Request {
        View view;

        std::promise<Snapshot> promise;

        Request(const View &view)
            : view(view)
        {}
    };

    void stop();

    void worker(int threadId, const SnapperConfig &config
                , const glsupport::egl::Device &device);

    std::vector<std::thread> threads_;
    std::atomic<bool> running_;

    std::queue<Request> requests_;
    std::condition_variable requestsCond_;
    std::mutex requestsMutex_;
};

}} // namespace vts::offscreen

#endif // vtsoffscreen_snapper_hpp_included_
