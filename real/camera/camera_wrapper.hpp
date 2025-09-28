#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <map>
#include <atomic>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>

using namespace libcamera;

class CameraWrapper
{
public:
    CameraWrapper(unsigned int width = 1920, unsigned int height = 1080)
        : width_(width), height_(height), running_(false) {}

    ~CameraWrapper() { stop(); }

    bool start()
    {
        cm_ = std::make_unique<CameraManager>();
        cm_->start();

        if (cm_->cameras().empty())
        {
            std::cerr << "No cameras found." << std::endl;
            return false;
        }

        camera_ = cm_->get(cm_->cameras()[0]->id());
        if (!camera_)
            return false;

        camera_->acquire();

        config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
        StreamConfiguration &sconf = config_->at(0);
        sconf.size.width = width_;
        sconf.size.height = height_;
        config_->validate();
        camera_->configure(config_.get());

        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
        for (StreamConfiguration &cfg : *config_)
        {
            allocator_->allocate(cfg.stream());
        }
        stream_ = config_->at(0).stream();

        const auto &buffers = allocator_->buffers(stream_);
        for (unsigned int i = 0; i < buffers.size(); ++i)
        {
            std::unique_ptr<Request> request = camera_->createRequest();
            request->addBuffer(stream_, buffers[i].get());
            requests_.push_back(std::move(request));
        }

        camera_->requestCompleted.connect(this, &CameraWrapper::requestComplete);

        ControlList controls(camera_->controls());

        controls.set(controls::FrameDuration, 33333);
        controls.set(controls::AwbEnable, true);
        controls.set(controls::AeEnable, false);
        controls.set(controls::ExposureTime, 15000);
        controls.set(controls::AnalogueGain, 8.0);

        camera_->start(&controls);

        for (auto &r : requests_)
            camera_->queueRequest(r.get());

        running_ = true;
        return true;
    }

    void stop()
    {
        if (!running_)
            return;
        camera_->stop();
        allocator_->free(stream_);
        camera_->release();
        camera_.reset();
        cm_->stop();
        running_ = false;
    }

    cv::Mat getFrame()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (latestFrame_.empty())
        {
            return cv::Mat(height_, width_, CV_8UC4);
        }
        return latestFrame_.clone();
    }

private:
    void requestComplete(Request *request)
    {
        if (request->status() == Request::RequestCancelled)
            return;

        for (auto &[stream, buffer] : request->buffers())
        {
            const FrameBuffer::Plane &plane = buffer->planes()[0];

            void *mem =
                mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
            if (mem == MAP_FAILED)
                continue;

            cv::Mat img(height_, width_, CV_8UC4, mem);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                img.copyTo(latestFrame_);
            }

            munmap(mem, plane.length);
        }

        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

    unsigned int width_, height_;
    bool running_;
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    Stream *stream_;
    std::vector<std::unique_ptr<Request>> requests_;

    std::mutex mutex_;
    cv::Mat latestFrame_;
};