#ifndef RcInput_h
#define RcInput_h
#include "math.h"
#include "config.h"
#include "mathFunctions.h"
#include "medianFilter.h"
#include <PPMReader.h>

class RcInput
{
public:
    enum InputType // The types of inputs
    {
        JOYSTICK,              // a normal joystick
        CENTER_JOYSTICK,       // a joystick that is centered
        SWITCH,                // a switch
        THREE_POSITION_SWITCH, // a switch that has three positions
        DIAL                   // a dial
    };
    bool invertOutput{false};                         // Whether the output should be inverted
    int inputChannel;                                 // The channel of the input
    int minInput{config::DEFAULT_RC_INPUT_MIN_INPUT}; // The minimum value the input can be
    int maxInput{config::DEFAULT_RC_INPUT_MAX_INPUT}; // The maximum value the input can be
    int centerInput{(minInput + maxInput) / 2};       // The center input value for a centered joystick or a  three position switch
    int minOutput{0};                                 // The minimum output value
    int maxOutput{100};                               // The maximum output value
    int deadzone{config::DEFAULT_RC_INPUT_DEADZONE};  // The deadzone for the input
    medianFilter *filter;                             // The median filter for the input
    InputType inputType;                              // The type of input

    volatile unsigned long input; // Used by the interrupt to captture when the input pin changes states. This is volatile because it is accessed by the interrupt.

    /**
     * Constructor
     *
     * @param inputType The type of input
     * @param inputChannel The pin the joystick is connected to
     * @param ppm The PPM reader
     */
    RcInput(InputType inputType, int inputChannel, medianFilter *filter)
    {
        this->filter = filter;
        this->inputType = inputType;
        this->inputChannel = inputChannel;
        switch (inputType)
        {
        case JOYSTICK:
            this->minOutput = 0;
            this->maxOutput = 100;
            break;
        case CENTER_JOYSTICK:
            this->minOutput = -100;
            this->maxOutput = 100;
            break;
        case SWITCH:
            this->minOutput = 0;
            this->maxOutput = 1;
            this->deadzone = 0;
            break;
        case THREE_POSITION_SWITCH:
            this->minOutput = -1;
            this->maxOutput = 1;
            this->deadzone = 0;
            break;
        case DIAL:
            this->minOutput = 0;
            this->maxOutput = 100;
            break;
        }
    }

    /**
     * Gets the current input
     *
     * @return The current input
     */
    int getCurrentInput()
    {
        return filter->getMedian();
    }

    /**
     * Calculates and updates the center input value
     *
     * @return The center input value
     */
    int getCenterInput()
    {
        centerInput = (minInput + maxInput) / 2;
        return centerInput;
    }

    /**
     * Gets the current output of the input.
     *
     * @return The mapped output. A HIGH or LOW value for a switch or a value between the rangeMin and rangeMax for a joystick or dial.
     */
    int getOutput()
    {
        int output = 0;
        switch (this->inputType)
        {
        case SWITCH:
            output = getSwitchOutput();
            break;
        case THREE_POSITION_SWITCH:
            output = getThreePositionSwitchOutput();
            break;
        case CENTER_JOYSTICK:
            output = getCenterJoystickOutput();
            break;
        case JOYSTICK:
            output = getJoystickOutput();
            break;
        case DIAL:
            output = getDialOutput();
            break;
        default:
            output = 0;
            break;
        }
        if (invertOutput)
        {
            output *= -1;
        }
        return output;
    }

private:
    /**
     * Gets the output for a non-centered joystick.
     *
     * @return The mapped output
     */
    int getJoystickOutput()
    {
        int output = getCurrentInput();
        if (output > maxInput)
            output = maxInput;
        if (output < minInput)
            output = minInput;
        if (abs(output) < deadzone)
            return 0;
        return mathFunctions::map(output, minInput, maxInput, minOutput, maxOutput);
    }

    /**
     * Gets the output for a dial
     *
     * @return The mapped output
     */
    int getDialOutput()
    {
        int output = getCurrentInput();
        if (output > maxInput)
            output = maxInput;
        if (output < minInput)
            output = minInput;
        if (abs(output) < deadzone)
            return 0;
        return mathFunctions::map(output, minInput, maxInput, minOutput, maxOutput);
    }

    /**
     * Gets the output for a joystick that is centered at the center input.
     *
     * @return The mapped output
     */
    int getCenterJoystickOutput()
    {
        int output = getCurrentInput();
        if (output > maxInput)
            output = maxInput;
        if (output < minInput)
            output = minInput;
        if (abs(getCurrentInput() - getCenterInput()) < deadzone)
            return 0;
        if (output < getCenterInput())
        {
            return mathFunctions::map(output, minInput, getCenterInput(), minOutput, 0);
        }
        else
        {
            return mathFunctions::map(getCurrentInput(), getCenterInput(), maxInput, 0, maxOutput);
        }
    }

    /**
     * Gets the output for a switch.
     *
     * @return HIGH or LOW
     */
    int getSwitchOutput()
    {
        if (getCurrentInput() > (maxInput + minInput) / 2)
        {
            return HIGH;
        }
        else
        {
            return LOW;
        }
    }

    /**
     * Gets the output for a switch that has three positions.
     *
     * @return -1, 0 or 1 based on the position of the switch
     */
    int getThreePositionSwitchOutput()
    {
        int highDivider{(maxInput - getCenterInput()) / 2};
        int lowDivider{(getCenterInput() - minInput) / 2};
        if (getCurrentInput() >= highDivider)
        {
            return 1;
        }
        if (getCurrentInput() < lowDivider)
        {
            return -1;
        }
        return 0;
    }
};

#endif
