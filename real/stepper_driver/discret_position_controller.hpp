#pragma once
#include <vector>
#include <stdexcept>
#include <cmath>
#include "stepper_controller.hpp"

class DiscretStepperController
{
private:
    StepperController stepperController;
    std::vector<int> currentPositions;

    // Configurables:
    int stepsPerRevolution;
    int minPosition;
    int maxPosition;
    int positionStep;

    int degreesToSteps(int degrees) const
    {
        return (degrees * stepsPerRevolution) / 360;
    }

    int stepsToDegrees(int steps) const
    {
        return (steps * 360) / stepsPerRevolution;
    }

    bool isValidPosition(int degrees) const
    {
        return degrees >= minPosition &&
               degrees <= maxPosition &&
               (degrees - minPosition) % positionStep == 0;
    }

public:
    DiscretStepperController(const std::vector<MotorPins> &motorPins,
                             int minDeg,
                             int maxDeg,
                             int stepDeg,
                             int delayMicroseconds = 100,
                             int microstepping = 16) // TMC2209 MS1=MS2=HIGH
        : stepperController(motorPins, delayMicroseconds),
          currentPositions(motorPins.size(), minDeg),
          stepsPerRevolution(200 * microstepping),
          minPosition(minDeg),
          maxPosition(maxDeg),
          positionStep(stepDeg)
    {
        if (stepDeg <= 0)
            throw std::invalid_argument("Position step must be > 0");
        if (minDeg >= maxDeg)
            throw std::invalid_argument("Invalid min/max position range");
    }

    void setMotorPosition(int motorIndex, int targetDegrees)
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");

        if (!isValidPosition(targetDegrees))
            throw std::invalid_argument("Invalid position");

        int currentDegrees = currentPositions[motorIndex];
        int deltaSteps = degreesToSteps(targetDegrees - currentDegrees);

        stepperController.moveMotor(motorIndex, deltaSteps);
        currentPositions[motorIndex] = targetDegrees;
    }

    void setAllMotorsPosition(int targetDegrees)
    {
        if (!isValidPosition(targetDegrees))
            throw std::invalid_argument("Invalid position");

        for (size_t i = 0; i < currentPositions.size(); i++)
            setMotorPosition(i, targetDegrees);
    }

    int getMotorPosition(int motorIndex) const
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");
        return currentPositions[motorIndex];
    }

    std::vector<int> getAllPositions() const
    {
        return currentPositions;
    }

    void homeMotor(int motorIndex)
    {
        setMotorPosition(motorIndex, minPosition);
    }

    void homeAllMotors()
    {
        setAllMotorsPosition(minPosition);
    }

    void incrementMotorPosition(int motorIndex)
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");

        int currentPos = currentPositions[motorIndex];
        if (currentPos + positionStep <= maxPosition)
            setMotorPosition(motorIndex, currentPos + positionStep);
    }

    void decrementMotorPosition(int motorIndex)
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");

        int currentPos = currentPositions[motorIndex];
        if (currentPos - positionStep >= minPosition)
            setMotorPosition(motorIndex, currentPos - positionStep);
    }

    int getMotorCount() const
    {
        return currentPositions.size();
    }

    // For convenience – report config
    void printConfig() const
    {
        std::cout << "Steps per revolution: " << stepsPerRevolution << "\n";
        std::cout << "Position range: " << minPosition << "° to " << maxPosition << "°\n";
        std::cout << "Position step size: " << positionStep << "°\n";
    }
};