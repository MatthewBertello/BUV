#ifndef RcInput_h
#define RcInput_h
#include "math.h"
#include "config.h"
#include "mathFunctions.h"
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
    int inputChannel;                                 // The channel of the input
    int minInput{config::DEFAULT_RC_INPUT_MIN_INPUT}; // The minimum value the input can be
    int maxInput{config::DEFAULT_RC_INPUT_MAX_INPUT}; // The maximum value the input can be
    int centerInput{(minInput + maxInput) / 2};       // The center input value for a centered joystick or a  three position switch
    int minOutput{0};                                 // The minimum output value
    int maxOutput{100};                               // The maximum output value
    int deadzone{config::DEFAULT_RC_INPUT_DEADZONE};  // The deadzone for the input
    PPMReader *ppm;                                   // The PPM reader
    InputType inputType;                              // The type of input

    volatile unsigned long input; // Used by the interrupt to captture when the input pin changes states. This is volatile because it is accessed by the interrupt.

    /**
     * Constructor
     *
     * @param inputType The type of input
     * @param inputChannel The pin the joystick is connected to
     * @param ppm The PPM reader
     */
    RcInput(InputType inputType, int inputChannel, PPMReader *ppm)
    {
        this->ppm = ppm;
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
        ppm->latestValidChannelValue(inputChannel, 0);
    }

    /**
     * Gets the current output of the input.
     *
     * @return The mapped output. A HIGH or LOW value for a switch or a value between the rangeMin and rangeMax for a joystick or dial.
     */
    int getOutput()
    {
        switch (this->inputType)
        {
        case SWITCH:
            return getSwitchOutput();
            break;
        case THREE_POSITION_SWITCH:
            return getThreePositionSwitchOutput();
            break;
        case CENTER_JOYSTICK:
            return getCenterJoystickOutput();
            break;
        case JOYSTICK:
            return getJoystickOutput();
            break;
        case DIAL:
            return getDialOutput();
            break;
        default:
            return 0;
            break;
        }
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
        if (abs(getCurrentInput() - centerInput) < deadzone)
            return 0;
        if (output < centerInput)
        {
            return mathFunctions::map(output, minInput, centerInput, minOutput, 0);
        }
        else
        {
            return mathFunctions::map(getCurrentInput(), centerInput, maxInput, 0, maxOutput);
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
     * @return -1, 0 or 1
     */
    int getThreePositionSwitchOutput()
    {
        int highDivider{(maxInput - centerInput) / 2};
        int lowDivider{(centerInput - minInput) / 2};
        if (getCurrentInput() >= highDivider)
        {
            return 1;
        }
        else if (getCurrentInput() < highDivider && getCurrentInput() > lowDivider)
        {
            return 0;
        }
        else
        {
            return -1;
        }
    }
};

#endif
