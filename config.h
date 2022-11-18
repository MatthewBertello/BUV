#ifndef config_h
#define config_h

namespace config
{
    // Input pins
    inline constexpr int PPM_INTERRUPT_PIN{2};            // The pin the PPM interrupt is connected to - must be an interrupt pin
    inline constexpr int FORWARD_SWITCH_PIN{34};          // The pin the forward switch is connected to
    inline constexpr int REVERSE_SWITCH_PIN{36};          // The pin the reverse switch is connected to
    inline constexpr int FOOT_SWITCH_PIN{38};             // The pin the tow switch is connected to
    inline constexpr int THROTTLE_HIHG_REFERENCE_PIN{40}; // The pin the throttle high reference is connected to
    inline constexpr int THROTTLE_WIPER_PIN{42};          // The pin the throttle wiper is connected to
    inline constexpr int THROTTLE_LOW_REFERENCE_PIN{44};  // The pin the throttle low reference is connected to

    // bourns encoder constants
    inline constexpr int LEFT_ENCODER_PIN_1{11}; // The left encoder pin 1
    inline constexpr int LEFT_ENCODER_PIN_2{19}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_3{13}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_4{14}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_5{15}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_6{21}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_7{17}; // The left encoder pin 2
    inline constexpr int LEFT_ENCODER_PIN_8{18}; // The left encoder pin 2

    // Input Channels
    inline constexpr int RIGHT_STICK_LEFT_RIGHT{1};  // The channel the right stick's left/right is connected to - RC CH1
    inline constexpr int RIGHT_STICK_UP_DOWN{2};     // The channel the right stick's up/down is connected to - RC CH2
    inline constexpr int LEFT_STICK_UP_DOWN{3};      // The channel the left stick's up/down is connected to - RC CH3
    inline constexpr int LEFT_STICK_LEFT_RIGHT{4};   // The channel the left stick's left/right is connected to - RC CH4
    inline constexpr int TOP_LEFT_SWITCH{5};         // The channel the top left switch is connected to - RC CH5
    inline constexpr int TOP_LEFT_CENTER_SWITCH{6};  // The channel the top left center switch is connected to - RC CH6
    inline constexpr int TOP_RIGHT_CENTER_SWITCH{7}; // The channel the top right center switch is connected to - RC CH7
    inline constexpr int TOP_RIGHT_SWITCH{8};        // The channel the top right switch is connected to - RC CH8
    inline constexpr int LEFT_DIAL{9};               // The channel the left dial is connected to - RC CH9
    inline constexpr int RIGHT_DIAL{10};             // The channel the right dial is connected to - RC CH10

    // Output pins
    inline constexpr uint8_t BRAKE_STEPPER_PULSE_PIN{24};    // The brake stepper pulse pin
    inline constexpr uint8_t BRAKE_STEPPER_DIR_PIN{25};      // The brake stepper direction pin
    inline constexpr uint8_t STEERING_STEPPER_PULSE_PIN{22}; // The steering stepper pulse pin
    inline constexpr uint8_t STEERING_STEPPER_DIR_PIN{23};   // The steering stepper direction pin
    inline constexpr int MAIN_MOTOR_OUPTUT_PIN{12};          // The main motor output pin
    inline constexpr int FOOT_SWITCH_OUTPUT_PIN{6};          // The foot switch output pin
    inline constexpr int TOW_SWITCH_OUTPUT_PIN{7};           // The tow switch output pin
    inline constexpr int FORWARD_SWITCH_OUTPUT_PIN{5};       // The forward switch output pin
    inline constexpr int REVERSE_SWITCH_OUTPUT_PIN{4};       // The reverse switch output pin

    // Stepper
    inline constexpr int DEFAULT_STEPPER_STEPS_PER_REVOLUTION{400};                            // The default number of steps per revolution
    inline constexpr int DEFAULT_STEPPER_MAX_SPEED{DEFAULT_STEPPER_STEPS_PER_REVOLUTION * 10}; // The default maximum speed
    inline constexpr int DEFAULT_STEPPER_ACCELERATION{DEFAULT_STEPPER_MAX_SPEED * 4};          // The default acceleration
    inline constexpr int DEFAULT_STEPPER_ERROR_THRESHOLD{10};                                  // The default error threshold

    // Brake stepper constants
    inline constexpr int BRAKE_STEPPER_STEPS_PER_REVOLUTION{400}; // The brake stepper number of steps per revolution
    inline constexpr int BRAKE_STEPPER_MAX_SPEED{800};            // The brake stepper maximum speed
    inline constexpr int BRAKE_STEPPER_ACCELERATION{800};         // The brake stepper acceleration
    inline constexpr int BRAKE_STEPPER_RANGE_MINIMUM{-109};       // The brake stepper minimum range
    inline constexpr int BRAKE_STEPPER_RANGE_MAXIMUM{109};        // The brake stepper maximum range
    inline constexpr int BRAKE_STEPPER_ERROR_THRESHOLD{10};       // The brake stepper error threshold

    // Steering stepper constants
    inline constexpr int STEERING_STEPPER_STEPS_PER_REVOLUTION{400}; // The steering stepper number of steps per revolution
    inline constexpr int STEERING_STEPPER_MAX_SPEED{800};            // The steering stepper maximum speed
    inline constexpr int STEERING_STEPPER_ACCELERATION{3200};        // The steering stepper acceleration
    inline constexpr int STEERING_STEPPER_RANGE_MINIMUM{-800};       // The steering stepper minimum range
    inline constexpr int STEERING_STEPPER_RANGE_MAXIMUM{800};        // The steering stepper maximum range
    inline constexpr int STEERING_STEPPER_ERROR_THRESHOLD{10};       // The steering stepper error threshold

    // BUV
    inline constexpr int INPUT_REFRESH_RATE{5};                       // How often to read the inputs in ms
    inline constexpr bool DISABLE_HOMING_MODE{false};                 // Disable the homing mode
    inline constexpr int MINIMUM_OUTPUT_FOR_MAIN_MOTOR_THROTTLE{165}; // The minimum output to apply power to the main motor throttle

    // RCInput
    inline constexpr int DEFAULT_RC_INPUT_MIN_INPUT{1000};                // The default minimum input value
    inline constexpr int DEFAULT_RC_INPUT_MAX_INPUT{2000};                // The default maximum input value
    inline constexpr int DEFAULT_RC_INPUT_DEADZONE{10};                   // The default deadzone
    inline constexpr int DEFAULT_RC_INPUT_MEDIAN_FILTER_SIZE{10};         // The default median filter size
    inline constexpr int DEFAULT_RC_INPUT_MEDIAN_FILTER_INITIAL_VALUE{0}; // The default median filter initial value

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
    inline constexpr bool STEERING_JOYSTICK_INVERTED{true};                                                               // If the steering joystick is inverted

    // Homing mode switch constants
    inline constexpr int HOMING_MODE_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The homing mode switch minimum input value
    inline constexpr int HOMING_MODE_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The homing mode switch maximum input value
    inline constexpr bool HOMING_MODE_SWITCH_INVERTED{false};                      // If the homing mode switch is inverted

    // Set home switch constants
    inline constexpr int SET_HOME_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The set home switch minimum input value
    inline constexpr int SET_HOME_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The set home switch maximum input value
    inline constexpr bool SET_HOME_SWITCH_INVERTED{false};                      // If the set home switch is inverted

    // Tow switch constants
    inline constexpr int TOW_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The tow switch minimum input value
    inline constexpr int TOW_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The tow switch maximum input value
    inline constexpr bool TOW_SWITCH_INVERTED{false};                      // If the tow switch is inverted

    // gear switch constants
    inline constexpr int GEAR_SWITCH_MIN_INPUT{DEFAULT_RC_INPUT_MIN_INPUT}; // The gear switch minimum input value
    inline constexpr int GEAR_SWITCH_MAX_INPUT{DEFAULT_RC_INPUT_MAX_INPUT}; // The gear switch maximum input value
    inline constexpr bool GEAR_SWITCH_INVERTED{true};                       // If the gear switch is inverted

} // namespace config

#endif
