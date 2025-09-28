#include <gpiod.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>

#define CHIP_NAME "gpiochip0"

struct MotorPins
{
    int dir;
    int step;
};

class StepperController
{
private:
    struct MotorContext
    {
        gpiod_line *stepLine;
        gpiod_line *dirLine;
        std::atomic<int> targetSteps{0}; // Steps to move
        std::atomic<int> currentPos{0};  // Current absolute position in steps
        std::atomic<bool> running{true}; // Thread control
        std::mutex mtx;
        std::condition_variable cv;
        std::thread worker;
    };

    std::vector<std::unique_ptr<MotorContext>> motors;
    int delayMicros;
    gpiod_chip *chip;

    void motorWorker(int motorIndex)
    {
        MotorContext &motor = *motors[motorIndex];

        while (motor.running)
        {
            std::unique_lock<std::mutex> lock(motor.mtx);
            motor.cv.wait(lock, [&]
                          { return motor.targetSteps != 0 || !motor.running; });

            if (!motor.running)
                break;

            int steps = motor.targetSteps.exchange(0);
            lock.unlock();

            executeMove(motorIndex, steps);
        }
    }

    void executeMove(int motorIndex, int steps)
    {
        MotorContext &motor = *motors[motorIndex];

        bool direction = steps > 0;
        steps = std::abs(steps);

        gpiod_line_set_value(motor.dirLine, direction ? 1 : 0);
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        for (int i = 0; i < steps && motor.running; i++)
        {
            // Check if new movement requested
            if (motor.targetSteps != 0)
                break;

            gpiod_line_set_value(motor.stepLine, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(delayMicros));
            gpiod_line_set_value(motor.stepLine, 0);

            if (direction)
                motor.currentPos++;
            else
                motor.currentPos--;

            std::this_thread::sleep_for(std::chrono::microseconds(delayMicros));
        }
    }

public:
    StepperController(const std::vector<MotorPins> &motorPins,
                      int delayMicroseconds = 100)
        : delayMicros(delayMicroseconds)
    {
        chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip)
            throw std::runtime_error("Failed to open GPIO chip");

        motors.reserve(motorPins.size());

        for (size_t i = 0; i < motorPins.size(); i++)
        {
            std::unique_ptr<MotorContext> motor = std::make_unique<MotorContext>();
            motor->stepLine = gpiod_chip_get_line(chip, motorPins[i].step);
            motor->dirLine = gpiod_chip_get_line(chip, motorPins[i].dir);

            if (!motor->stepLine || !motor->dirLine)
            {
                throw std::runtime_error("Failed to get GPIO lines");
            }
            if (gpiod_line_request_output(motor->stepLine, "stepper", 0) < 0 ||
                gpiod_line_request_output(motor->dirLine, "stepper", 0) < 0)
            {
                throw std::runtime_error("Failed to request GPIO lines");
            }

            motor->worker = std::thread(&StepperController::motorWorker, this, i);
            motors.push_back(std::move(motor));
        }
    }

    ~StepperController()
    {
        for (std::unique_ptr<MotorContext> &motor : motors)
        {
            motor->running = false;
            motor->cv.notify_all();
            if (motor->worker.joinable())
            {
                motor->worker.join();
            }
        }
        if (chip)
            gpiod_chip_close(chip);
    }

    void moveMotor(int motorIndex, int steps)
    {
        if (motorIndex >= motors.size())
            return;
        MotorContext &motor = *motors[motorIndex];
        motor.targetSteps = steps;
        motor.cv.notify_one();
    }

    void moveAllMotors(int steps)
    {
        for (size_t i = 0; i < motors.size(); i++)
        {
            moveMotor(i, steps);
        }
    }

    // Get current absolute position in steps
    int getCurrentPosition(int motorIndex) const
    {
        if (motorIndex >= motors.size())
            return 0;
        return motors[motorIndex]->currentPos;
    }
};