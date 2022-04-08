#ifndef RcInput_h
#define RcInput_h
#include "math.h"
#include "config.h"
#include "mathFunctions.h"

class RcInput
{
public:
    enum InputType // The types of inputs
    {
        JOYSTICK,        // a normal joystick
        CENTER_JOYSTICK, // a joystick that is centered
        SWITCH,          // a switch
        DIAL             // a dial
    };
    int inputPin;                                     // The pin the input is connected to
    int minInput{config::DEFAULT_RC_INPUT_MIN_INPUT}; // The minimum value the input can be
    int maxInput{config::DEFAULT_RC_INPUT_MAX_INPUT}; // The maximum value the input can be
    int centerInput{(minInput + maxInput) / 2};       // The center input value for a centered joystick
    int minOutput{0};                                 // The minimum output value
    int maxOutput{100};                               // The maximum output value
    int deadzone{config::DEFAULT_RC_INPUT_DEADZONE};  // The deadzone for the input
    InputType inputType;                              // The type of input

    volatile unsigned long input; // Used by the interrupt to captture when the input pin changes states. This is volatile because it is accessed by the interrupt.

    int currentInput; // The current input value

    /**
     * Constructor
     *
     * @param inputType The type of input
     * @param inputPin The pin the joystick is connected to
     */
    RcInput(InputType inputType, int inputPin)
    {
        this->inputType = inputType;
        this->inputPin = inputPin;
        pinMode(inputPin, INPUT);
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
        case DIAL:
            this->minOutput = 0;
            this->maxOutput = 100;
            break;
        }
    }

    /**
     * Checks the state of the input pin. If the pin is high the time is recorded. If the pin is low the time difference is calculated and the current input is set to the difference.
     */
    void pinStateChange()
    {
        if (digitalRead(this->inputPin) == HIGH)
        {
            this->input = micros();
        }
        if (digitalRead(this->inputPin) == LOW)
        {
            this->currentInput = micros() - this->input;
        }
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
        if (currentInput > maxInput)
            currentInput = maxInput;
        if (currentInput < minInput)
            currentInput = minInput;
        if (abs(currentInput) < deadzone)
            return 0;
        return mathFunctions::map(currentInput, minInput, maxInput, minOutput, maxOutput);
    }

    /**
     * Gets the output for a dial
     *
     * @return The mapped output
     */
    int getDialOutput()
    {
        if (currentInput > maxInput)
            currentInput = maxInput;
        if (currentInput < minInput)
            currentInput = minInput;
        if (abs(currentInput) < deadzone)
            return 0;
        return mathFunctions::map(currentInput, minInput, maxInput, minOutput, maxOutput);
    }

    /**
     * Gets the output for a joystick that is centered at the center input.
     *
     * @return The mapped output
     */
    int getCenterJoystickOutput()
    {
        if (currentInput > maxInput)
            currentInput = maxInput;
        if (currentInput < minInput)
            currentInput = minInput;
        if (abs(currentInput - centerInput) < deadzone)
            return 0;
        if (currentInput < centerInput)
        {
            return mathFunctions::map(currentInput, minInput, centerInput, minOutput, 0);
        }
        else
        {
            return mathFunctions::map(currentInput, centerInput, maxInput, 0, maxOutput);
        }
    }

    /**
     * Gets the output for a switch.
     *
     * @return HIGH or LOW
     */
    int getSwitchOutput()
    {
        if (currentInput > (maxInput + minInput) / 2)
        {
            return HIGH;
        }
        else
        {
            return LOW;
        }
    }
};

#endif
