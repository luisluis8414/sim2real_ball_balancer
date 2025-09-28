#include "camera/frame_stack.hpp"
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

int main()
{
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ball_balancer");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(4);
    session_options.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);

    const char *model_path = "models/version_2/policy2.onnx";
    Ort::Session session(env, model_path, session_options);

    const char *input_name = "input";
    const char *output_name = "actions";

    std::vector<int64_t> input_shape{1, 12, 64, 64};
    size_t input_tensor_size = 1 * 12 * 64 * 64;

    FrameStack fs(64, 64);
    if (!fs.start())
    {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }

    // Create output directory
    std::string dir = "image_dump";
    fs::create_directories(dir);

    Ort::AllocatorWithDefaultOptions allocator;
    int frame_id = 0;

    while (true)
    {
        cv::Mat stack = fs.getNextFrameStack();
        if (stack.empty())
        {
            std::cerr << "Got empty frame stack" << std::endl;
            continue;
        }
        bool log_img = false;
        if (log_img)
        {
            std::string filepath = dir + "/stack_" + std::to_string(frame_id++) + ".png";
            cv::imwrite(filepath, stack);
            std::cout << "Saved " << filepath
                      << " (" << stack.cols << "x" << stack.rows << ")" << std::endl;
        }

        cv::Mat stack_rgb;
        cv::cvtColor(stack, stack_rgb, cv::COLOR_BGRA2RGB);

        std::vector<float> input_tensor_values(input_tensor_size);

        for (int f = 0; f < 4; f++)
        {
            cv::Rect roi(f * 64, 0, 64, 64);
            cv::Mat frame64 = stack_rgb(roi);

            for (int y = 0; y < 64; y++)
            {
                for (int x = 0; x < 64; x++)
                {
                    cv::Vec3b pix = frame64.at<cv::Vec3b>(y, x);
                    for (int c = 0; c < 3; c++)
                    {
                        int channel = f * 3 + c; // (0..11)
                        size_t idx = channel * 64 * 64 + y * 64 + x;
                        input_tensor_values[idx] = pix[c] / 255.0f;
                    }
                }
            }
        }

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtDeviceAllocator, OrtMemTypeCPU);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor_values.data(), input_tensor_values.size(),
            input_shape.data(), input_shape.size());

        std::vector<Ort::Value> output_tensors = session.Run(
            Ort::RunOptions{nullptr}, &input_name, &input_tensor, 1,
            &output_name, 1);

        float *output_data = output_tensors.front().GetTensorMutableData<float>();

        std::vector<float> actions(3);
        for (int i = 0; i < 3; i++)
            actions[i] = std::tanh(output_data[i]);

        std::cout << "Raw: ["
                  << output_data[0] << ", "
                  << output_data[1] << ", "
                  << output_data[2] << "]  "
                  << "Tanh-squashed: ["
                  << actions[0] << ", "
                  << actions[1] << ", "
                  << actions[2] << "]"
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fs.stop();
    return 0;
}