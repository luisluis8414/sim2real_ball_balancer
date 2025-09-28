#include "camera_wrapper.hpp"
#include <deque>
#include <opencv2/opencv.hpp>

class FrameStack
{
public:
    FrameStack(int width = 64, int height = 64)
        : width_(width), height_(height), initialized_(false) {}

    bool start()
    {
        if (!cam_.start())
            return false;

        // Warm-up: drop first 10 frames
        int skipped = 0;
        while (skipped < 10)
        {
            cv::Mat f = cam_.getFrame();
            if (!f.empty())
            {
                skipped++;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        return true;
    }

    void stop() { cam_.stop(); }

    cv::Mat getNextFrameStack()
    {
        cv::Mat frame = cam_.getFrame();
        if (frame.empty())
            return cv::Mat();

        cv::Mat cropped = cropCenterSquare(frame);

        cv::Mat resized;
        cv::resize(cropped, resized, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);

        if (!initialized_)
        {
            // First call: fill stack with the same frame repeated
            for (int i = 0; i < 4; ++i)
                frames_.push_front(resized.clone());
            initialized_ = true;
        }
        else
        {
            // Shift: insert new frame left, pop right if >4
            frames_.push_front(resized.clone());
            if (frames_.size() > 4)
                frames_.pop_back();
        }

        cv::Mat stack;
        cv::hconcat(std::vector<cv::Mat>(frames_.begin(), frames_.end()), stack);
        return stack;
    }

private:
    cv::Mat cropCenterSquare(const cv::Mat &frame)
    {
        int h = frame.rows;
        int w = frame.cols;
        int size = std::min(h, w);
        int x0 = (w - size) / 2;
        int y0 = (h - size) / 2;
        cv::Rect roi(x0, y0, size, size);
        return frame(roi);
    }

    int width_, height_;
    CameraWrapper cam_;
    std::deque<cv::Mat> frames_;
    bool initialized_;
};