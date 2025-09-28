#include <iostream>
#include <vector>
#include <random>
#include <thread>
#include <chrono>
#include "continuous_position_controller.hpp"

int main()
{
    std::vector<MotorPins> pins = {
        {26, 19},
        {21, 20}};

    try
    {
        ContinuousStepperController controller(pins, 0, 120, 100, 16);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        const int minDeg = 0;
        const int maxDeg = 120;

        auto endTime = std::chrono::steady_clock::now() + std::chrono::seconds(60);

        while (std::chrono::steady_clock::now() < endTime)
        {
            std::vector<double> targets;
            for (int i = 0; i < pins.size(); i++)
            {
                double r = dist(gen); // [0,1]
                double degrees = minDeg + r * (maxDeg - minDeg);
                targets.push_back(degrees);
            }

            controller.setMotorPositions(targets);

            // Sleep 100ms → 10Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "\nTime's up! Homing all motors...\n";
        controller.homeAllMotors();

        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Program completed.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}