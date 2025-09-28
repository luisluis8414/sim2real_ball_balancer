#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/libcamera.h>

#include <opencv2/opencv.hpp>

#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>

using namespace libcamera;
using namespace std::chrono_literals;

static void requestComplete(Request *request);

static std::shared_ptr<Camera> camera;

int main()
{

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    for (auto const &camera : cm->cameras())
        std::cout << camera->id() << std::endl;

    auto cameras = cm->cameras();
    if (cameras.empty())
    {
        std::cout << "No cameras were identified on the system."
                  << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    std::string cameraId = cameras[0]->id();

    camera = cm->get(cameraId);

    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});

    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;

    streamConfig.size.width = 64;
    streamConfig.size.height = 64;

    config->validate();
    std::cout << "Validated viewfinder configuration is: " << streamConfig.toString() << std::endl;

    camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

    for (StreamConfiguration &cfg : *config)
    {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0)
        {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }

        auto &buffers = allocator->buffers(cfg.stream());
        std::cout << "Allocated " << buffers.size()
                  << " buffers for stream" << std::endl;

        for (size_t i = 0; i < buffers.size(); i++)
        {
            FrameBuffer *buffer = buffers[i].get();
            size_t totalSize = 0;
            for (const FrameBuffer::Plane &plane : buffer->planes())
            {
                totalSize += plane.length;
            }
            std::cout << "Buffer " << i << " size: "
                      << totalSize << " bytes" << std::endl;
        }
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;

    for (unsigned int i = 0; i < buffers.size(); ++i)
    {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return -ENOMEM;
        }

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << "Can't set buffer for request"
                      << std::endl;
            return ret;
        }

        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);

    camera->start();

    for (std::unique_ptr<Request> &request : requests)
        camera->queueRequest(request.get());

    std::this_thread::sleep_for(3000ms);

    camera->stop();
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    return 0;
}

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    for (auto bufferPair : buffers)
    {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        const FrameBuffer::Plane &fbPlane = buffer->planes()[0];

        void *mem = mmap(NULL, fbPlane.length,
                         PROT_READ, MAP_SHARED,
                         fbPlane.fd.get(), 0);
        if (mem == MAP_FAILED)
        {
            perror("mmap");
            continue;
        }

        cv::Mat img(64, 64, CV_8UC4, mem);

        std::string filename = "frame-" + std::to_string(metadata.sequence) + ".png";
        cv::imwrite(filename, img);

        munmap(mem, fbPlane.length);

        std::cout << "Saved " << filename << std::endl;
    }

    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}