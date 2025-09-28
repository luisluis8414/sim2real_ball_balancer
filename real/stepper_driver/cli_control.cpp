#include <iostream>
#include <string>
#include <sstream>
#include "continuous_position_controller.hpp"

int main()
{
    std::vector<MotorPins> pins = {
        {26, 19},
        {21, 20},
        {13, 6}};

    try
    {
        // Initialize continuous controller: range [0°, 120]
        ContinuousStepperController controller(pins, 0, 120, 200);

        std::cout << "Continuous Motor Control CLI\n";
        std::cout << "Commands:\n";
        std::cout << "  set <motorIndex> <degrees>   - Move a motor to angle (any degree in range: 0-230)\n";
        std::cout << "  set all <degrees>            - Move ALL motors\n";
        std::cout << "  home [motorIndex|all]        - Move to minimum position (0°)\n";
        std::cout << "  pos                          - Show all motor positions\n";
        std::cout << "  quit                         - Exit program\n\n";

        std::string line;
        while (true)
        {
            std::cout << "> ";
            if (!std::getline(std::cin, line))
                break;

            std::istringstream iss(line);
            std::string cmd;
            iss >> cmd;

            if (cmd == "quit" || cmd == "exit")
            {
                break;
            }
            else if (cmd == "set")
            {
                std::string target;
                double degrees;
                iss >> target >> degrees;

                if (!iss)
                {
                    std::cerr << "Usage: set <motorIndex|all> <degrees>\n";
                    continue;
                }

                try
                {
                    if (target == "all")
                    {
                        controller.setAllMotorsPosition(degrees);
                    }
                    else
                    {
                        int idx = std::stoi(target);
                        controller.setMotorPosition(idx, degrees);
                    }
                }
                catch (std::exception &e)
                {
                    std::cerr << "Error: " << e.what() << "\n";
                }
            }
            else if (cmd == "home")
            {
                std::string target;
                iss >> target;
                if (target == "all" || target.empty())
                {
                    controller.homeAllMotors();
                    std::cout << "Homed all motors\n";
                }
                else
                {
                    try
                    {
                        int idx = std::stoi(target);
                        controller.homeMotor(idx);
                        std::cout << "Homed motor " << idx << "\n";
                    }
                    catch (std::exception &e)
                    {
                        std::cerr << "Error: " << e.what() << "\n";
                    }
                }
            }
            else if (cmd == "pos")
            {
                auto positions = controller.getAllPositions();
                for (size_t i = 0; i < positions.size(); i++)
                {
                    std::cout << "Motor " << i
                              << " at " << positions[i] << "°\n";
                }
            }
            else if (!cmd.empty())
            {
                std::cerr << "Unknown command: " << cmd << "\n";
            }
        }

        std::cout << "Exiting program.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
