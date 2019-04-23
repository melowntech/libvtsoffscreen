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

#include <dlfcn.h>

#include <limits>

#include <vts-browser/map.hpp>
#include <vts-browser/mapStatistics.hpp>
#include <vts-browser/mapCallbacks.hpp>
#include <vts-browser/camera.hpp>
#include <vts-browser/navigation.hpp>
#include <vts-browser/navigationOptions.hpp>
#include <vts-browser/resources.hpp>
#include <vts-browser/fetcher.hpp>
#include <vts-renderer/renderer.hpp>

#include "dbglog/dbglog.hpp"

#include "glsupport/egl.hpp"
#include "utility/gl.hpp"

#ifdef VTSOFFSCREEN_HAS_OPTICS
#include "optics/cameragladaptor.hpp"
#endif

#include "geo/csconvertor.hpp"

#include "snapper.hpp"

extern "C" {

void *snapper_cpp_getGlProcAddress(const char *name)
{
    ::dlerror();
    auto sym(::dlsym(RTLD_DEFAULT, name));
    if (auto error = ::dlerror()) {
        LOG(warn1)
            << "Unable to get address of OpenGL function <"
            << name << ">: <" << error << ">.";
    }
    return sym;
}

} // extern "C"

namespace vts { namespace offscreen {

namespace {

/** Creates EGL context and makes it current.
*   Returned value must be kept to work.
 */
glsupport::egl::Context
eglContext(const glsupport::egl::Device &device = glsupport::egl::Device()
           , const math::Size2 &size = math::Size2(1, 1))
{
    // open display
    auto dpy(device
             ? glsupport::egl::Display(device)
             : glsupport::egl::Display());

    if (!::eglBindAPI(EGL_OPENGL_API)) {
        LOGTHROW(err2, glsupport::egl::Error)
            << "EGL: Cannot bind OpenGL API ("
            << glsupport::egl::detail::error() << ").";
    }

    // choose EGL configuration
    // the vts renderer manages its own buffer, so we do not need any 'bits'
    const auto configs(chooseConfigs(dpy, {
                EGL_SURFACE_TYPE, EGL_PBUFFER_BIT
                , EGL_CONFORMANT, EGL_OPENGL_BIT
                , EGL_BLUE_SIZE, 0
                , EGL_GREEN_SIZE, 0
                , EGL_RED_SIZE, 0
                , EGL_ALPHA_SIZE, 0
                , EGL_DEPTH_SIZE, 0
                , EGL_STENCIL_SIZE, 0
                , EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT
                , EGL_NONE
            }));

    // create 1x1 surface, we do not use it for output
    auto surface(pbuffer(dpy, configs, {
                EGL_WIDTH, size.width
                , EGL_HEIGHT, size.height
                , EGL_NONE
          }));

    auto ctx(context(dpy, configs, {
                EGL_CONTEXT_MAJOR_VERSION, 3
                , EGL_CONTEXT_MINOR_VERSION, 3
                , EGL_CONTEXT_OPENGL_PROFILE_MASK,
                         EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT
#ifndef NDEBUG
                , EGL_CONTEXT_OPENGL_DEBUG, EGL_TRUE
#endif
                , EGL_NONE
             }));
    ctx.makeCurrent(surface);
    return ctx;
}

int loadGlFunctions()
{
    vts::renderer::loadGlFunctions(&snapper_cpp_getGlProcAddress);
    return 0;
}

void waitForGl()
{
    auto fence(::glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0));
    if (!fence) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot create fence.";
    }
    while (::glClientWaitSync(fence, GL_SYNC_FLUSH_COMMANDS_BIT, 500000000UL)
           == GL_TIMEOUT_EXPIRED)
    {
        LOG(info1) << "GL: still waiting for fence.";
    }

    ::glDeleteSync(fence);

    glFinish();
    glFinish();
}

class Fetcher : public vts::Fetcher {
public:
    Fetcher(const FetcherOptions &options)
        : f_(vts::Fetcher::create(options))
    {}

    virtual void initialize() override { f_->initialize(); }

    virtual void finalize() override { f_->finalize(); }

    virtual void fetch(const std::shared_ptr<FetchTask> &task) override
    {
        class FetchTask : public vts::FetchTask {
        public:
            FetchTask(const std::shared_ptr<vts::FetchTask> &task)
                : vts::FetchTask(task->query)
                , task_(task)
            {}

            virtual void fetchDone() {
                // move stuff back
                task_->reply = std::move(reply);
                // override expires
                task_->reply.expires
                    = std::numeric_limits<decltype(reply.expires)>::max();
                task_->fetchDone();
            }

        private:
            std::shared_ptr<vts::FetchTask> task_;
        };

        return f_->fetch(std::make_shared<FetchTask>(task));
    }

private:
    std::shared_ptr<vts::Fetcher> f_;
};

} // namespace

SnapperConfig::SnapperConfig()
    : antialiasingSamples(0), renderAtmosphere(true)
{
    confMapCreate.clientId = "vts-snapper";
    confMapRuntime.targetResourcesMemoryKB = 1500000l;
    confCamera.traverseModeSurfaces = vts::TraverseMode::Flat;
    confCamera.traverseModeGeodata = vts::TraverseMode::Flat;
    confMapRuntime.maxResourceProcessesPerTick
        = std::numeric_limits
        <decltype(confMapRuntime.maxResourceProcessesPerTick)>::max();
    confMapRuntime.fetchFirstRetryTimeOffset = 1;
}

Snapshot::Snapshot(const math::Size2 &size)
    : image(size.height, size.width, cv::Vec3b(0, 0, 255))
{}

class Snapper::Detail {
public:
    Detail(const SnapperConfig &config);
    Detail(const SnapperConfig &config, const glsupport::egl::Device &device);

    ~Detail();

    Snapshot snap(const View &view);

private:
    Detail(const glsupport::egl::Context &ctx, const SnapperConfig &config);

    glsupport::egl::Context ctx_;
    const SnapperConfig config_;
    std::shared_ptr<vts::Map> map_;
    std::shared_ptr<vts::Camera> camera_;
    std::shared_ptr<vts::Navigation> navigation_;
    std::shared_ptr<vts::renderer::RenderContext> context_;
    std::shared_ptr<vts::renderer::RenderView> view_;
    bool mcReady_;
};

Snapper::Detail::Detail(const glsupport::egl::Context &ctx,
                        const SnapperConfig &config)
    : ctx_(ctx)
    , config_(config)
    , mcReady_(false)
{
    static int dummyLoadGlFunctions = loadGlFunctions(); // just once
    (void)dummyLoadGlFunctions;

    LOG(info3)
        << "Using OpenGL device: vendor: <" << ::glGetString(GL_VENDOR)
        << ">, renderer: <" << ::glGetString(GL_RENDERER)
        << ">, version: <" << ::glGetString(GL_VERSION)
        << ">.";

    // setup opengl context
    vts::renderer::installGlDebugCallback();
    ::glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // create map, camera and navigation
    {
        vts::MapCreateOptions mco = config.confMapCreate;
        mco.customSrs1 = config.customSrs1.toString();
        mco.customSrs2 = config.customSrs2.toString();
        map_ = std::make_shared<vts::Map>(mco,
                    std::make_shared<Fetcher>(FetcherOptions()));
        camera_ = map_->createCamera();
        navigation_ = camera_->createNavigation();

        // apply options
        map_->options()= config.confMapRuntime;
        camera_->options() = config.confCamera;

        // navigation options
        auto &no(navigation_->options());
        no.navigationType = vts::NavigationType::Instant;
        no.cameraNormalization = false;
        no.cameraAltitudeChanges = false;
        no.viewExtentLimitScaleMin = 0.0;
        no.viewExtentLimitScaleMax = std::numeric_limits<double>::max();
        no.tiltLimitAngleLow = 0.0;
        no.tiltLimitAngleHigh = 360.0;
    }

    // create render context and render view
    {
        context_ = std::make_shared<vts::renderer::RenderContext>();
        context_->bindLoadFunctions(map_.get());
        view_ = context_->createView(camera_.get());
    }

    // get notified when mapconfig is ready
    {
        auto &cb(map_->callbacks());
        // no need for shared pointer since map is member of this class
        cb.mapconfigReady = [=]() mutable {
            mcReady_ = true;
        };
    }

    // initialize map
    map_->dataInitialize();
    map_->renderInitialize();
    map_->setMapconfigPath(config.mapConfigUrl, config.authUrl);

    /** Pump until we have valid map config
     */
    while (!mcReady_) {
        map_->dataUpdate();
        map_->renderUpdate(0);
        ::usleep(20);
    }

    // map config is ready we can proceed
    LOG(info2) << "Mapconfig is ready, we can proceed";
}

Snapper::Detail::Detail(const SnapperConfig &config)
    : Detail(eglContext(), config)
{}

Snapper::Detail::Detail(const SnapperConfig &config
                        , const glsupport::egl::Device &device)
    : Detail(eglContext(device), config)
{}

Snapper::Detail::~Detail()
{
    map_->dataFinalize();
    map_->renderFinalize();
}
namespace {

class PositionSetter : public boost::static_visitor<>
{
public:
    PositionSetter(vts::Map &map, vts::Navigation &nav)
        : map_(map), nav_(nav) {}

    void operator()(const VtsJsonPosition &position) const {
        nav_.setPositionJson(position.position);
    }

    void operator()(const VtsSerializedPosition &position) const {
        nav_.setPositionUrl(position.position);
    }

#ifdef VTSOFFSCREEN_HAS_OPTICS
    // available only when compiled with liboptics
    void operator()(const OpticsPosition &position) const;
#endif

    /** Helper for matrix-like point transformation. Converts homogeneous point
     *  between from Custom1 SRS to Physical SRS.
     */
    math::Point4 prod1(const math::Point4 &p) const;

private:
    vts::Map &map_;
    vts::Navigation &nav_;
};

inline math::Point4 PositionSetter::prod1(const math::Point4 &p) const
{
    double in[3] = { p[0] / p[3], p[1] / p[3], p[2] / p[3] };
    math::Point4 out;
    map_.convert(in, &out[0], vts::Srs::Custom1, vts::Srs::Physical);
    out[3] = 1.0;
    return out;
}

inline math::Point4 prod(const PositionSetter &ps, const math::Point4 &p)
{
    return ps.prod1(p);
}

#ifdef VTSOFFSCREEN_HAS_OPTICS
// available only when compiled with liboptics
void PositionSetter::operator()(const OpticsPosition &position) const {
    // transform position to destination SRS
    const auto pos(position.position.transform(*this));

    // we have to set position via callbacks
    auto &mc(map_.callbacks());

    mc.cameraOverrideView = [&pos](double *mat) {
        optics::CameraGLAdaptor::glViewMatrix(pos, mat);
    };

    mc.cameraOverrideProj = [&position](double *mat) {
        optics::CameraGLAdaptor::glProjectionMatrix
        (position.camera, 10.0, 100000.0, mat);
    };
}
#endif

inline void setPosition(vts::Map &map,
                vts::Navigation &nav, const Position &position)
{
    boost::apply_visitor(PositionSetter(map, nav), position);
}

} // namespace

Snapshot Snapper::Detail::snap(const View &view)
{
    const auto screenSize(view.viewport.size());
    camera_->setViewportSize(screenSize.width, screenSize.height);

    auto &ro(view_->options());
    ro.width = screenSize.width;
    ro.height = screenSize.height;
    ro.colorToTargetFrameBuffer = false;
    ro.colorToTexture = true;
    ro.antialiasingSamples = config_.antialiasingSamples;
    ro.renderAtmosphere = config_.renderAtmosphere;

    // set position
    for (int i = 0; i < 3; i++)
    {
        setPosition(*map_, *navigation_, view.position);
        // ensure the position change takes effect
        map_->renderUpdate(0);
        camera_->renderUpdate();
        map_->dataUpdate();
    }

    // wait till we have all resources for rendering
    {
        int row = 0;
        // repeat until two succesive renders
        //   fully complete (do not request any more resources)
        while (row < 2)
        {
            int cnt = 0;
            do {
                map_->renderUpdate(0);
                camera_->renderUpdate();
                map_->dataUpdate();
                ::usleep(20);
                cnt++;
            } while (!map_->getMapRenderComplete());
            if (cnt > 1)
                row = 0;
            else
                row++;
            // rendering may request more resources (eg. font textures)
            view_->render();
        }
    }

    // wait for gpu to finish
    waitForGl();

    // grab rendered image
    Snapshot snapshot = Snapshot(screenSize);

    // use render framebuffer and fetch image data (NB: BGR)
    const auto &rv(view_->variables());
    ::glBindFramebuffer(GL_READ_FRAMEBUFFER, rv.frameReadBufferId);
    vts::renderer::checkGlFramebuffer(GL_READ_FRAMEBUFFER);
    void *dst = (void*)snapshot.image.data;
    assert(dst);
    vts::renderer::checkGlImpl("1");
    ::glReadPixels(0, 0, screenSize.width, screenSize.height
                   , GL_BGR, GL_UNSIGNED_BYTE, dst);
    vts::renderer::checkGlImpl("snapper");
    cv::flip(snapshot.image, snapshot.image, false);

    // keypoints
    for (const auto &keypoint : view.keypoints) {
        math::Point3 world;
        view_->getWorldPosition(&keypoint[0], &world[0]);
        if (std::isnan(world(0))
            || std::isnan(world(1))
            || std::isnan(world(2)))
        {
            // invalid point
            continue;
        }
        map_->convert(&world[0], &world[0], vts::Srs::Physical
                     , vts::Srs::Custom1);
        snapshot.keypoints.emplace_back(keypoint, world);
    }

    return snapshot;
}

Snapper::Snapper(const SnapperConfig &config)
    : detail_(std::make_unique<Detail>(config))
{}

Snapper::Snapper(const SnapperConfig &config
                 , const glsupport::egl::Device &device)
    : detail_(std::make_unique<Detail>(config, device))
{}

Snapper::~Snapper() {}

Snapshot Snapper::snap(const View &view)
{
    return detail_->snap(view);
}

AsyncSnapper::AsyncSnapper(const SnapperConfig &config)
    : running_(false)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard([this]() { stop(); });

    try {
        const auto devices(glsupport::egl::queryDevices());
        int threadId((devices.size() == 1) ? -1 : 0);
        for (const auto &device : devices) {
        threads_.emplace_back
            (&AsyncSnapper::worker, this, threadId++, config, device);
        }
    } catch (glsupport::egl::MissingExtension) {
        LOG(warn2)
            << "Cannot probe available devices (extension unavailable). "
            << "Running on default native display.";
        threads_.emplace_back
            (&AsyncSnapper::worker, this
             , -1, config, glsupport::egl::Device());
    }

    guard.release();
    running_ = true;

    // TODO: add barrier to wait for snapper generation in the thread
}

void AsyncSnapper::stop()
{
    running_ = false;
    requestsCond_.notify_all();
    for (auto &thread : threads_) {
        thread.join();
    }
}

AsyncSnapper::~AsyncSnapper()
{
    stop();
}

Snapshot AsyncSnapper::operator()(const View &view)
{
    std::unique_lock<std::mutex> lock(requestsMutex_);
    requests_.emplace(view);
    auto future(requests_.back().promise.get_future());
    requestsCond_.notify_all();
    lock.unlock();
    return future.get();
}

void AsyncSnapper::worker(int threadId
                          , const SnapperConfig &config
                          , const glsupport::egl::Device &device)
{
    dbglog::thread_id((threadId < 0)
                      ? std::string("snapper")
                      : str(boost::format("snapper:%d") % threadId));

    Snapper snapper(config, device);

    while (running_) {
        // BEGIN critical section
        std::unique_lock<std::mutex> lock(requestsMutex_);

        if (requests_.empty()) {
            if (!running_) { break; }
            requestsCond_.wait(lock);
            continue;
        }

        auto request(std::move(requests_.front()));
        requests_.pop();
        // unlock
        lock.unlock();
        // END critical section

        try {
            request.promise.set_value(snapper.snap(request.view));
        } catch (...) {
            request.promise.set_exception(std::current_exception());
        }
    }
}

}} // namespace vts::offscreen

