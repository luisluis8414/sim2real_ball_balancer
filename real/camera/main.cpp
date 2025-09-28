#include "frame_stack.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>

namespace fs = std::filesystem;

int main()
{
    std::string dir = "image_dump";
    fs::create_directories(dir);

    FrameStack fs(64, 64);
    if (!fs.start())
        return -1;

    for (int i = 0; i < 25; ++i)
    {
        auto t1 = std::chrono::high_resolution_clock::now();

        cv::Mat stack = fs.getNextFrameStack();

        if (!stack.empty())
        {
            std::string filepath = dir + "/stack_" + std::to_string(i) + ".png";
            cv::imwrite(filepath, stack);
            std::cout << "Saved " << filepath
                      << " (" << stack.cols << "x" << stack.rows << ") " << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fs.stop();
    return 0;
}