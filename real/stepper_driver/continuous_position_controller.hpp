#pragma once
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include "stepper_controller.hpp"

class ContinuousStepperController
{
private:
    StepperController stepperController;
    std::vector<int> currentPositions; // Current positions in degrees

    // Configurables:
    int stepsPerRevolution; // e.g. 200 * microstepping
    int minPosition;        // Minimum position in degrees
    int maxPosition;        // Maximum position in degrees
    double degreesPerStep;  // Resolution based on steps per revolution

    int degreesToSteps(double degrees) const
    {
        return static_cast<int>(std::round(degrees / degreesPerStep));
    }

    double stepsToDegrees(int steps) const
    {
        return steps * degreesPerStep;
    }

public:
    ContinuousStepperController(const std::vector<MotorPins> &motorPins,
                                int minDeg,
                                int maxDeg,
                                int delayMicroseconds = 100,
                                int microstepping = 16) // TMC2209 MS1=MS2=HIGH
        : stepperController(motorPins, delayMicroseconds),
          currentPositions(motorPins.size(), minDeg),
          stepsPerRevolution(200 * microstepping),
          minPosition(minDeg),
          maxPosition(maxDeg),
          degreesPerStep(360.0 / stepsPerRevolution)
    {
        if (minDeg >= maxDeg)
            throw std::invalid_argument("Invalid min/max position range");
    }

    void setMotorPosition(int motorIndex, double targetDegrees)
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");

        if (targetDegrees > maxPosition || targetDegrees < minPosition)
        {
            std::cout << "invalid position" << std::endl;
            return;
        }

        int currentSteps = stepperController.getCurrentPosition(motorIndex);

        // Calculate needed movement based on actual position
        int targetSteps = degreesToSteps(targetDegrees);
        int deltaSteps = targetSteps - currentSteps;

        if (deltaSteps != 0)
        {
            stepperController.moveMotor(motorIndex, deltaSteps);
        }
    }

    // Add method to get actual position in degrees
    double getActualPosition(int motorIndex) const
    {
        if (motorIndex >= currentPositions.size())
            throw std::out_of_range("Motor index out of range");

        int currentSteps = stepperController.getCurrentPosition(motorIndex);
        return stepsToDegrees(currentSteps);
    }

    void setAllMotorsPosition(double targetDegrees)
    {
        for (size_t i = 0; i < currentPositions.size(); i++)
            setMotorPosition(i, targetDegrees);
    }

    void setMotorPositions(const std::vector<double> &targetDegrees)
    {
        if (targetDegrees.size() != currentPositions.size())
            throw std::invalid_argument("Target positions vector size must match motor count");

        for (size_t i = 0; i < targetDegrees.size(); i++)
            setMotorPosition(i, targetDegrees[i]);
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
};