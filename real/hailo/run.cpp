#include <hailo/hailort.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <cstring> // for std::memcpy

int main()
{
    const std::string hef_path = "./policy.hef";

    // ==========================
    // Device
    // ==========================
    hailort::Expected<std::unique_ptr<hailort::VDevice>> vdevice_exp = hailort::VDevice::create();
    if (!vdevice_exp)
    {
        std::cerr << "Failed to open Hailo device: "
                  << vdevice_exp.status() << std::endl;
        return 1;
    }
    std::unique_ptr<hailort::VDevice> vdevice = vdevice_exp.release();

    // ==========================
    // Load HEF
    // ==========================
    hailort::Expected<hailort::Hef> hef_exp = hailort::Hef::create(hef_path);
    if (!hef_exp)
    {
        std::cerr << "Failed to load HEF: " << hef_exp.status() << "\n";
        return 1;
    }
    hailort::Hef hef = hef_exp.release();

    // ==========================
    // Network group
    // ==========================
    std::vector<std::string> ng_names = hef.get_network_groups_names();
    if (ng_names.empty())
    {
        std::cerr << "No network groups in HEF.\n";
        return 1;
    }
    std::string chosen_group = ng_names[0];

    hailort::Expected<hailort::ConfigureNetworkParams> cfg_exp =
        hef.create_configure_params(HAILO_STREAM_INTERFACE_PCIE, chosen_group);
    if (!cfg_exp)
    {
        std::cerr << "Failed to create configure params: " << cfg_exp.status() << "\n";
        return 1;
    }
    hailort::ConfigureNetworkParams configure_params = cfg_exp.release();

    hailort::NetworkGroupsParamsMap cfg_map;
    cfg_map[chosen_group] = configure_params;

    hailort::Expected<hailort::ConfiguredNetworkGroupVector> networks_exp =
        vdevice->configure(hef, cfg_map);
    if (!networks_exp)
    {
        std::cerr << "Failed to configure: " << networks_exp.status() << "\n";
        return 1;
    }
    hailort::ConfiguredNetworkGroupVector networks = networks_exp.release();
    std::shared_ptr<hailort::ConfiguredNetworkGroup> network_group_ptr = networks.at(0);
    hailort::ConfiguredNetworkGroup &network_group = *network_group_ptr;

    // ==========================
    // Get valid vstream params
    // ==========================
    // Example values:
    bool unused = false;
    hailo_format_type_t format_type = HAILO_FORMAT_TYPE_AUTO; // let SDK choose appropriate format
    uint32_t timeout_ms = 1000;                               // 1 second timeout
    uint32_t queue_size = 5;                                  // typical default >0

    // Generate params
    hailort::Expected<std::map<std::string, hailo_vstream_params_t>> input_params_exp =
        network_group.make_input_vstream_params(unused, format_type, timeout_ms, queue_size);

    hailort::Expected<std::map<std::string, hailo_vstream_params_t>> output_params_exp =
        network_group.make_output_vstream_params(unused, format_type, timeout_ms, queue_size);

    if (!input_params_exp || !output_params_exp)
    {
        std::cerr << "Failed to create vstream params" << std::endl;
        return 1;
    }

    std::map<std::string, hailo_vstream_params_t> input_params = input_params_exp.release();
    std::map<std::string, hailo_vstream_params_t> output_params = output_params_exp.release();

    // ==========================
    // Create vstreams
    // ==========================
    hailort::Expected<std::vector<hailort::InputVStream>> input_vstreams_exp =
        hailort::VStreamsBuilder::create_input_vstreams(network_group, input_params);
    hailort::Expected<std::vector<hailort::OutputVStream>> output_vstreams_exp =
        hailort::VStreamsBuilder::create_output_vstreams(network_group, output_params);

    if (!input_vstreams_exp || !output_vstreams_exp)
    {
        std::cerr << "Failed to create vstreams: "
                  << (input_vstreams_exp ? output_vstreams_exp.status()
                                         : input_vstreams_exp.status())
                  << std::endl;
        return 1;
    }

    std::vector<hailort::InputVStream> input_vstreams = std::move(input_vstreams_exp.release());
    std::vector<hailort::OutputVStream> output_vstreams = std::move(output_vstreams_exp.release());

    if (input_vstreams.empty() || output_vstreams.empty())
    {
        std::cerr << "No input/output vstreams created.\n";
        return 1;
    }

    hailort::InputVStream &in_stream = input_vstreams.at(0);
    hailort::OutputVStream &out_stream = output_vstreams.at(0);

    // ==========================
    // Load input image with OpenCV
    // ==========================
    cv::Mat frame = cv::imread("debug_images/first_frame_stack_000025.png");
    if (frame.empty())
    {
        std::cerr << "No test image found.\n";
        return 1;
    }

    // Make sure it's 256x64 (4 stacked frames of 64x64)
    cv::resize(frame, frame, cv::Size(256, 64));
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    // Split into 4 subframes (each 64x64x3)
    std::vector<cv::Mat> subframes;
    for (int i = 0; i < 4; i++)
    {
        cv::Rect roi(i * 64, 0, 64, 64);
        subframes.push_back(frame(roi).clone());
    }

    // Collect all 12 channels
    std::vector<cv::Mat> channels;
    for (auto &f : subframes)
    {
        std::vector<cv::Mat> tmp;
        cv::split(f, tmp);                                       // splits into [R,G,B]
        channels.insert(channels.end(), tmp.begin(), tmp.end()); // 3 per frame → 12 total
    }

    // Merge into one 12-channel Mat: (64x64x12)
    cv::Mat merged;
    cv::merge(channels, merged);

    // Copy into contiguous buffer
    std::vector<uint8_t> input_data(merged.total() * merged.channels());
    std::memcpy(input_data.data(), merged.data, input_data.size());

    hailort::MemoryView input_buffer(input_data.data(), input_data.size());

    // ==========================
    // Write input into Hailo
    // ==========================
    hailo_status status = in_stream.write(input_buffer);
    if (status != HAILO_SUCCESS)
    {
        std::cerr << "Write failed: " << status << "\n";
        return 1;
    }

    // ==========================
    // Read output
    // ==========================
    std::vector<uint8_t> output_data(out_stream.get_frame_size());
    hailort::MemoryView output_buffer(output_data.data(), output_data.size());

    status = out_stream.read(output_buffer);
    if (status != HAILO_SUCCESS)
    {
        std::cerr << "Read failed: " << status << "\n";
        return 1;
    }

    std::cout << "Inference done! Output size = "
              << output_data.size() << " bytes" << std::endl;

    std::cout << "Output values: [ ";
    for (size_t i = 0; i < output_data.size(); i++)
    {
        // cast to int so it prints numbers, not chars
        std::cout << static_cast<int>(output_data[i]) << " ";
    }
    std::cout << "]" << std::endl;

    const std::vector<hailo_quant_info_t> &quant_infos = out_stream.get_quant_infos();

    if (!quant_infos.empty())
    {
        float scale = quant_infos[0].qp_scale;
        int zero_point = quant_infos[0].qp_zp;

        std::cout << "Quantization: scale=" << scale
                  << " zero_point=" << zero_point << std::endl;

        std::cout << "Dequantized values: [ ";
        for (size_t i = 0; i < output_data.size(); i++)
        {
            int raw = static_cast<int>(output_data[i]);
            float real_val = scale * (raw - zero_point);
            std::cout << real_val << " ";
        }
        std::cout << "]" << std::endl;
    }

    return 0;
}