#ifndef config_h
#define config_h

namespace config
{
    // Input pins
    inline constexpr int GAS_JOYSTICK_INPUT_PIN{2};             // The pin the gas joystick is connected to - must be an interrupt pin - RC CH2
    inline constexpr int STEERING_JOYSTICK_INPUT_PIN{3};        // The pin the steering joystick is connected to - must be an interrupt pin - RC CH4
    inline constexpr int TOP_LEFT_SWITCH_INPUT_PIN{18};         // The pin the top left switch is connected to - must be an interrupt pin -
    inline constexpr int TOP_RIGHT_SWITCH_INPUT_PIN{19};        // The pin the top right switch is connected to - must be an interrupt pin
    inline constexpr int TOP_LEFT_CENTER_SWITCH_INPUT_PIN{20};  // The pin the top left center switch is connected to - must be an interrupt pin
    inline constexpr int TOP_RIGHT_CENTER_SWITCH_INPUT_PIN{21}; // The pin the top right center switch is connected to - must be an interrupt pin

    // Output pins
    inline constexpr uint8_t BRAKE_STEPPER_PULSE_PIN{24};    // The brake stepper pulse pin
    inline constexpr uint8_t BRAKE_STEPPER_DIR_PIN{25};      // The brake stepper direction pin
    inline constexpr uint8_t STEERING_STEPPER_PULSE_PIN{22}; // The steering stepper pulse pin
    inline constexpr uint8_t STEERING_STEPPER_DIR_PIN{23};   // The steering stepper direction pin
    inline constexpr int MAIN_MOTOR_OUPTUT_PIN{11};          // The main motor output pin
    inline constexpr int FOOT_SWITCH_OUTPUT_PIN{9};          // The foot switch output pin
    inline constexpr int TOW_SWITCH_OUTPUT_PIN{10};          // The tow switch output pin
    inline constexpr int FORWARD_SWITCH_OUTPUT_PIN{12};      // The forward switch output pin
    inline constexpr int REVERSE_SWITCH_OUTPUT_PIN{13};      // The reverse switch output pin

    // Stepper
    inline constexpr int DEFAULT_STEPPER_STEPS_PER_REVOLUTION{400};                            // The default number of steps per revolution
    inline constexpr int DEFAULT_STEPPER_MAX_SPEED{DEFAULT_STEPPER_STEPS_PER_REVOLUTION * 10}; // The default maximum speed
    inline constexpr int DEFAULT_STEPPER_ACCELERATION{DEFAULT_STEPPER_MAX_SPEED * 4};          // The default acceleration
    inline constexpr int DEFAULT_STEPPER_ERROR_THRESHOLD{10};                                  // The default error threshold

    // Brake stepper constants
    inline constexpr int BRAKE_STEPPER_STEPS_PER_REVOLUTION{DEFAULT_STEPPER_STEPS_PER_REVOLUTION}; // The brake stepper number of steps per revolution
    inline constexpr int BRAKE_STEPPER_MAX_SPEED{DEFAULT_STEPPER_MAX_SPEED};                       // The brake stepper maximum speed
    inline constexpr int BRAKE_STEPPER_ACCELERATION{DEFAULT_STEPPER_ACCELERATION};                 // The brake stepper acceleration
    inline constexpr int BRAKE_STEPPER_RANGE_MINIMUM{-BRAKE_STEPPER_STEPS_PER_REVOLUTION};         // The brake stepper minimum range
    inline constexpr int BRAKE_STEPPER_RANGE_MAXIMUM{BRAKE_STEPPER_STEPS_PER_REVOLUTION};          // The brake stepper maximum range
    inline constexpr int BRAKE_STEPPER_ERROR_THRESHOLD{DEFAULT_STEPPER_ERROR_THRESHOLD};           // The brake stepper error threshold

    // Steering stepper constants
    inline constexpr int STEERING_STEPPER_STEPS_PER_REVOLUTION{DEFAULT_STEPPER_STEPS_PER_REVOLUTION}; // The steering stepper number of steps per revolution
    inline constexpr int STEERING_STEPPER_MAX_SPEED{DEFAULT_STEPPER_MAX_SPEED};                       // The steering stepper maximum speed
    inline constexpr int STEERING_STEPPER_ACCELERATION{DEFAULT_STEPPER_ACCELERATION};                 // The steering stepper acceleration
    inline constexpr int STEERING_STEPPER_RANGE_MINIMUM{-STEERING_STEPPER_STEPS_PER_REVOLUTION};      // The steering stepper minimum range
    inline constexpr int STEERING_STEPPER_RANGE_MAXIMUM{STEERING_STEPPER_STEPS_PER_REVOLUTION};       // The steering stepper maximum range
    inline constexpr int STEERING_STEPPER_ERROR_THRESHOLD{DEFAULT_STEPPER_ERROR_THRESHOLD};           // The steering stepper error threshold

    // BUV
    inline constexpr int INPUT_REFRESH_RATE{5};                       // How often to read the inputs in ms
    inline constexpr bool DISABLE_HOMING_MODE{false};                 // Disable the homing mode
    inline constexpr int MINIMUM_OUTPUT_FOR_MAIN_MOTOR_THROTTLE{165}; // The minimum output to apply power to the main motor throttle

    // RCInput
    inline constexpr int DEFAULT_RC_INPUT_MIN_INPUT{1000}; // The default minimum input value
    inline constexpr int DEFAULT_RC_INPUT_MAX_INPUT{2000}; // The default maximum input value
    inline constexpr int DEFAULT_RC_INPUT_DEADZONE{10};    // The default deadzone

    // Gas joystick constants
    inline constexpr int GAS_JOYSTICK_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT};                               // The gas joystick minimum input value
    inline constexpr int GAS_JOYSTICK_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT};                               // The gas joystick maximum input value
    inline constexpr int GAS_JOYSTICK_DEADZONE{DEFAULT_RC_INPUT_DEADZONE};                                 // The gas joystick deadzone
    inline constexpr int GAS_JOYSTICK_CENTER_VALUE{(GAS_JOYSTICK_MAX_INPUT + GAS_JOYSTICK_MIN_INPUT) / 2}; // The gas joystick center value

    // Steering joystick constants
    inline constexpr int STEERING_JOYSTICK_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT};                                         // The steering joystick minimum input value
    inline constexpr int STEERING_JOYSTICK_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT};                                         // The steering joystick maximum input value
    inline constexpr int STEERING_JOYSTICK_DEADZONE{DEFAULT_RC_INPUT_DEADZONE};                                           // The steering joystick deadzone
    inline constexpr int STEERING_JOYSTICK_CENTER_VALUE{(STEERING_JOYSTICK_MAX_INPUT + STEERING_JOYSTICK_MIN_INPUT) / 2}; // The steering joystick center value

    // Homing mode switch constants
    // The max and min input values are swapped because the input value is low when the switch is in the up position
    inline constexpr int HOMING_MODE_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The homing mode switch minimum input value
    inline constexpr int HOMING_MODE_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The homing mode switch maximum input value

    // Top left center switch constants
    // The max and min input values are swapped because the input value is low when the switch is in the up position
    inline constexpr int TOP_LEFT_CENTER_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The top left center switch minimum input value
    inline constexpr int TOP_LEFT_CENTER_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The top left center switch maximum input value

    // Tow switch constants
    // The max and min input values are swapped because the input value is low when the switch is in the up position
    inline constexpr int TOW_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The tow switch minimum input value
    inline constexpr int TOW_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The tow switch maximum input value

    // gear switch constants
    // The max and min input values are swapped because the input value is low when the switch is in the up position
    inline constexpr int GEAR_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MAX_INPUT};                             // The gear switch minimum input value
    inline constexpr int GEAR_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MIN_INPUT};                             // The gear switch maximum input value
    inline constexpr int GEAR_SWITCH_CENTER_VALUE{(GEAR_SWITCH_MAX_INPUT + GEAR_SWITCH_MIN_INPUT) / 2}; // The gear switch center value

} // namespace config

#endif