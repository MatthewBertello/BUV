#ifndef Stepper_h
#define Stepper_h
#include <AccelStepper.h>
#include "config.h"
#include "mathFunctions.h"

class Stepper : public AccelStepper
{
public:
    int errorThreshold{config::DEFAULT_STEPPER_ERROR_THRESHOLD};          // The error threshold for the stepper. If the error is less than this value, the runThreshold function will not move the stepper.
    int stepsPerRevolution{config::DEFAULT_STEPPER_STEPS_PER_REVOLUTION}; // The number of steps per revolution for the stepper
    int rangeMinimum{-stepsPerRevolution};                                // The minimum range for the stepper
    int rangeMaximum{stepsPerRevolution};                                 // The maximum range for the stepper

    /**
     * Constructor
     *
     * @param stepsPerRevolution The number of steps per revolution
     * @param pulsePin The pin the stepper motor is connected to
     * @param dirPin The pin the stepper motor direction is connected to
     */
    Stepper(uint8_t interface = AccelStepper::FULL4WIRE,
            uint8_t pulsePin = 2,
            uint8_t dirPin = 3)
        : AccelStepper(interface, pulsePin, dirPin)
    {
        AccelStepper::setMaxSpeed(config::DEFAULT_STEPPER_MAX_SPEED);
        AccelStepper::setAcceleration(config::DEFAULT_STEPPER_ACCELERATION);
    }

    /**
     * If the distance to the target is less than the run threshold, then the target position is set to the current position and the run function is called. Otherwise the run function is called.
     */
    void runThreshold()
    {
        if (abs(AccelStepper::distanceToGo()) < errorThreshold)
        {
            AccelStepper::moveTo(AccelStepper::currentPosition());
        }
        AccelStepper::run();
    }

    /**
     * Sets the target position for the stepper.
     *
     * @param target The target position as a percentage of the range of the stepper
     */
    void moveToInRange(double target)
    {
        int targetPosition{mathFunctions::map(target, -100, 100, rangeMinimum, rangeMaximum)};

        AccelStepper::moveTo(targetPosition);
    }

    /**
     * Changes the target position for the stepper by the given percentage of it's total range.
     *
     * @param target How much to change the target position by as a percentage of the total range
     */
    void moveInRange(double target)
    {
        int targetPosition{int((double(rangeMaximum - rangeMinimum) / 100.0) * target)};
        AccelStepper::move(targetPosition);
    }

    /**
     * Runs the stepper at a given speed.
     *
     * @param speed The percentage of a revolution that the stepper should move per second
     */
    void runSpeedPercentage(double speed)
    {
        AccelStepper::setSpeed(int((double(stepsPerRevolution) / 100.0) * speed));
        AccelStepper::runSpeed();
    }
};
#endif