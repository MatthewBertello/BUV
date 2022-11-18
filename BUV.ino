#include <PPMReader.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ACE128.h>

#include "RcInput.h"
#include "Stepper.h"
#include "config.h"
#include "utilities.h"
<<<<<<< Updated upstream
=======
#include "PID.h"
#include "BournsEncoder.h"
>>>>>>> Stashed changes

#include "medianFilter.h"

unsigned long lastInputCheck{0}; // The last time the inputs were checked
enum Steppers                    // A list of the steppers
{
  BRAKE,
  STEERING,
};

PPMReader ppm(config::PPM_INTERRUPT_PIN, 10); // The PPM reader
Steppers stepperHoming = BRAKE;               // The stepper that is currently being homed

// Setup the RC inputs
RcInput gasJoystick{RcInput::CENTER_JOYSTICK, config::RIGHT_STICK_UP_DOWN};          // The joystick for the gas/brake
RcInput steeringJoystick{RcInput::CENTER_JOYSTICK, config::LEFT_STICK_LEFT_RIGHT};   // The joystick for the steering
RcInput homingModeSwitch{RcInput::SWITCH, config::TOP_LEFT_SWITCH};                  // The homing mode switch
RcInput setHomeSwitch{RcInput::SWITCH, config::TOP_LEFT_CENTER_SWITCH};              // The set home switch
RcInput towSwitch{RcInput::SWITCH, config::TOP_RIGHT_SWITCH};                        // The tow switch
RcInput gearSwitch{RcInput::THREE_POSITION_SWITCH, config::TOP_RIGHT_CENTER_SWITCH}; // The forward/reverse Switch

// Setup the stepper motors
Stepper brakeStepper{AccelStepper::DRIVER, config::BRAKE_STEPPER_PULSE_PIN, config::BRAKE_STEPPER_DIR_PIN};          // The stepper motor for the brake
Stepper steeringStepper{AccelStepper::DRIVER, config::STEERING_STEPPER_PULSE_PIN, config::STEERING_STEPPER_DIR_PIN}; // The stepper motor for the steering wheel

// Setup the Bourns encoders
BournsEncoder leftEncoder{config::LEFT_ENCODER_PIN_1,
                          config::LEFT_ENCODER_PIN_2,
                          config::LEFT_ENCODER_PIN_3,
                          config::LEFT_ENCODER_PIN_4,
                          config::LEFT_ENCODER_PIN_5,
                          config::LEFT_ENCODER_PIN_6,
                          config::LEFT_ENCODER_PIN_7,
                          config::LEFT_ENCODER_PIN_8};

// !This code is for debugging purposes only
// unsigned long lastPrint;
// unsigned long printRate{500};
//!

/**
 * Runs once before the main loop
 */
void setup()
{

  // !This code is for debugging purposes only
  Serial.begin(9600); // Start the serial port
  Serial.println("Start");

  straightDrive.setTarget(0);

  // Setup the output pins
  pinMode(config::MAIN_MOTOR_OUPTUT_PIN, OUTPUT);
  analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, 0);
  pinMode(config::TOW_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, 0);
  pinMode(config::FOOT_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, 0);
  pinMode(config::FORWARD_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, 0);
  pinMode(config::REVERSE_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, 0);

  // Setup the gas joystick
  gasJoystick.minInput = config::GAS_JOYSTICK_MIN_INPUT;
  gasJoystick.maxInput = config::GAS_JOYSTICK_MAX_INPUT;
  gasJoystick.deadzone = config::GAS_JOYSTICK_DEADZONE;
  gasJoystick.centerInput = config::GAS_JOYSTICK_CENTER_VALUE;

  // Setup the steering joystick
  steeringJoystick.minInput = config::STEERING_JOYSTICK_MIN_INPUT;
  steeringJoystick.maxInput = config::STEERING_JOYSTICK_MAX_INPUT;
  steeringJoystick.deadzone = config::STEERING_JOYSTICK_DEADZONE;
  steeringJoystick.invertOutput = config::STEERING_JOYSTICK_INVERTED;

  // Setup the homing mode switch
  homingModeSwitch.minInput = config::HOMING_MODE_SWITCH_MIN_INPUT;
  homingModeSwitch.maxInput = config::HOMING_MODE_SWITCH_MAX_INPUT;
  homingModeSwitch.invertOutput = config::HOMING_MODE_SWITCH_INVERTED;

  // Setup the set home switch
  setHomeSwitch.minInput = config::SET_HOME_SWITCH_MIN_INPUT;
  setHomeSwitch.maxInput = config::SET_HOME_SWITCH_MAX_INPUT;
  setHomeSwitch.invertOutput = config::SET_HOME_SWITCH_INVERTED;

  // Setup the tow switch
  towSwitch.minInput = config::TOW_SWITCH_MIN_INPUT;
  towSwitch.maxInput = config::TOW_SWITCH_MAX_INPUT;
  towSwitch.invertOutput = config::TOW_SWITCH_INVERTED;

  // Setup the gear switch
  gearSwitch.minInput = config::GEAR_SWITCH_MIN_INPUT;
  gearSwitch.maxInput = config::GEAR_SWITCH_MAX_INPUT;
  gearSwitch.invertOutput = config::GEAR_SWITCH_INVERTED;

  // Setup the brake stepper
  brakeStepper.stepsPerRevolution = config::BRAKE_STEPPER_STEPS_PER_REVOLUTION;
  brakeStepper.setMaxSpeed(config::BRAKE_STEPPER_MAX_SPEED);
  brakeStepper.setAcceleration(config::BRAKE_STEPPER_ACCELERATION);
  brakeStepper.rangeMinimum = config::BRAKE_STEPPER_RANGE_MINIMUM;
  brakeStepper.rangeMaximum = config::BRAKE_STEPPER_RANGE_MAXIMUM;
  brakeStepper.errorThreshold = config::BRAKE_STEPPER_ERROR_THRESHOLD;

  // Setup the steering stepper
  steeringStepper.stepsPerRevolution = config::STEERING_STEPPER_STEPS_PER_REVOLUTION;
  steeringStepper.setMaxSpeed(config::STEERING_STEPPER_MAX_SPEED);
  steeringStepper.setAcceleration(config::STEERING_STEPPER_ACCELERATION);
  steeringStepper.rangeMinimum = config::STEERING_STEPPER_RANGE_MINIMUM;
  steeringStepper.rangeMaximum = config::STEERING_STEPPER_RANGE_MAXIMUM;
  steeringStepper.errorThreshold = config::STEERING_STEPPER_ERROR_THRESHOLD;
}

/**
 * The main loop
 */
void loop()
{

<<<<<<< Updated upstream
  // Serial.println("Loop");
  //  !This code is for debugging purposes only
  //  Print current values
  // if (millis() - lastPrint > printRate)
  //{
  // lastPrint = millis();
  // for (int i = 1; i <= 10; i++)
  //{
  // int x = ppm.latestValidChannelValue(i, 0);
  //  int x = getPpmValue(i);

  // Serial.print(x);
  // Serial.print(" ");
  //}
  // gasJoystick.filter.print();
  // Serial.println(gasJoystick.getCurrentInput());
  //  gasJoystick.filter.print();
  //  Serial.println();
  //  Serial.print("gasJoystick input: ");
  //  Serial.print(gasJoystick.getCurrentInput());
  //  Serial.print(" ");
  // Serial.print("gas: ");
  // Serial.print(gasJoystick.getOutput());
  // Serial.print(" ");

  // Serial.print("steering: ");
  // Serial.print(steeringJoystick.getOutput());
  // Serial.print(" ");
  //  Serial.print(" ");
  //  Serial.print("brakeStepper position: ");
  //  Serial.print(brakeStepper.currentPosition());
  //  Serial.print(" ");
  //  Serial.print("brakeStepper target Position: ");
  //  Serial.print(brakeStepper.targetPosition());
  //  Serial.print(" ");
  //  Serial.print("accel distance: ");
  //  Serial.print(brakeStepper.distanceToGo());
  //  Serial.println();
  //  Serial.print("gearSwitch input: ");
  //  Serial.print(gearSwitch.getCurrentInput());
  //  Serial.print(" ");
  // Serial.print("gear: ");
  // Serial.print(gearSwitch.getOutput());
  // Serial.print(" ");
  //  Serial.print(" ");
  // Serial.print("tow: ");
  // Serial.print(towSwitch.getOutput());
  // Serial.print(" ");
  // Serial.print("homeMode: ");
  // Serial.print(homingModeSwitch.getOutput());
  // Serial.print(" ");
  // Serial.print("setHome: ");
  // Serial.print(setHomeSwitch.getOutput());

  // Serial.println();
  //}
  if (homingModeSwitch.getOutput() && !config::DISABLE_HOMING_MODE) // If the top left switch is off run the homing function
  {
    // Serial.println("start");
    homingMode();
    // Serial.println("end");
  }
  else // Otherwise run the normal operation
  {
    // if it has been long enough since the last input check get the new inputs
    if (millis() - lastInputCheck >= config::INPUT_REFRESH_RATE)
    {
      lastInputCheck = millis(); // Update the last input check time

      //! This code is for debugging purposes only
      unsigned long start = millis();
      //!

      updateFilters();

      // Get the input for the gas joystick and output it to the motor
      int gasInput{gasJoystick.getOutput()};
      if (gearSwitch.getOutput() == 1 && gasInput < 0)
      {
        gasInput = 0;
      }
      else if (gearSwitch.getOutput() == -1 && gasInput > 0)
      {
        gasInput = 0;
      }
      else if (gearSwitch.getOutput() == 0)
      {
        gasInput = 0;
      }
      if (gasInput != 0)
      {
        digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, HIGH);
        analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, utilities::map(abs(gasInput), 0, 100, config::MINIMUM_OUTPUT_FOR_MAIN_MOTOR_THROTTLE, 255));
      }
      else
      {
        digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, LOW);
        analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, 0);
      }
      // End of gas input

      // Get the input for the brake joystick and output it to the motor
      if ((gearSwitch.getOutput() == 1 || gearSwitch.getOutput() == 0) && gasJoystick.getOutput() < 0)
      {
        brakeStepper.moveToInRange(-gasJoystick.getOutput());
      }
      else if (gearSwitch.getOutput() == -1 && gasJoystick.getOutput() > 0)
      {
        brakeStepper.moveToInRange(gasJoystick.getOutput());
      }
      else
      {
        brakeStepper.moveToInRange(0);
      }
      // End of brake input

      steeringStepper.moveToInRange(steeringJoystick.getOutput());         // Set the target for the steering stepper
      digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, !towSwitch.getOutput()); // Set the tow switch output

      // set the gear switch output
      if (gearSwitch.getOutput() == 0)
      {
        digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, LOW);
        digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, LOW);
      }
      else if (gearSwitch.getOutput() == 1)
      {
        digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, HIGH);
        digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, LOW);
      }
      else if (gearSwitch.getOutput() == -1)
      {
        digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, LOW);
        digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, HIGH);
      }
      //! This code is for debugging purposes only
      // Serial.println(millis() - start);
      //!
    }

    // Run the stepper motors
    brakeStepper.runThreshold();
    steeringStepper.runThreshold();
  }
} // End of main loop
=======
  Serial.println(leftEncoder.getPosition());
  // delay(1000);
  // leftEncoder.recordPositions();
}
//   if (homingModeSwitch.getOutput() && !config::DISABLE_HOMING_MODE) // If the top left switch is off run the homing function
//   {
//     // Serial.println("start");
//     homingMode();
//     // Serial.println("end");
//   }
//   else // Otherwise run the normal operation
//   {
//     // if it has been long enough since the last input check get the new inputs
//     if (millis() - lastInputCheck >= config::INPUT_REFRESH_RATE)
//     {
//       lastInputCheck = millis(); // Update the last input check time

//       //! This code is for debugging purposes only
//       unsigned long start = millis();
//       //!

//       updateFilters();

//       // Get the input for the gas joystick and output it to the motor
//       int gasInput{gasJoystick.getOutput()};
//       if (gearSwitch.getOutput() == 1 && gasInput < 0)
//       {
//         gasInput = 0;
//       }
//       else if (gearSwitch.getOutput() == -1 && gasInput > 0)
//       {
//         gasInput = 0;
//       }
//       else if (gearSwitch.getOutput() == 0)
//       {
//         gasInput = 0;
//       }
//       if (gasInput != 0)
//       {
//         digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, HIGH);
//         analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, utilities::map(abs(gasInput), 0, 100, config::MINIMUM_OUTPUT_FOR_MAIN_MOTOR_THROTTLE, 255));
//       }
//       else
//       {
//         digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, LOW);
//         analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, 0);
//       }
//       // End of gas input

//       // Get the input for the brake joystick and output it to the motor
//       if ((gearSwitch.getOutput() == 1 || gearSwitch.getOutput() == 0) && gasJoystick.getOutput() < 0)
//       {
//         brakeStepper.moveToInRange(-gasJoystick.getOutput());
//       }
//       else if (gearSwitch.getOutput() == -1 && gasJoystick.getOutput() > 0)
//       {
//         brakeStepper.moveToInRange(gasJoystick.getOutput());
//       }
//       else
//       {
//         brakeStepper.moveToInRange(0);
//       }
//       // End of brake input

//       if (setHomeSwitch.getOutput())
//       {
//         driveStraight();
//       }
//       else
//       {
//         steeringStepper.moveToInRange(steeringJoystick.getOutput()); // Set the target for the steering stepper
//       }
//       digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, !towSwitch.getOutput()); // Set the tow switch output

//       // set the gear switch output
//       if (gearSwitch.getOutput() == 0)
//       {
//         digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, LOW);
//         digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, LOW);
//       }
//       else if (gearSwitch.getOutput() == 1)
//       {
//         digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, HIGH);
//         digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, LOW);
//       }
//       else if (gearSwitch.getOutput() == -1)
//       {
//         digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, LOW);
//         digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, HIGH);
//       }
//       //! This code is for debugging purposes only
//       // Serial.println(millis() - start);
//       //!
//     }

//     // Run the stepper motors
//     brakeStepper.runThreshold();
//     steeringStepper.runThreshold();
//   }
// } // End of main loop
>>>>>>> Stashed changes

/**
 * A function to set the home positions of the steppers
 */
void homingMode()
{
  updateFilters();
  // Select which stepper to home
  if (steeringJoystick.getOutput() > 50)
  {
    stepperHoming = BRAKE;
  }
  else if (steeringJoystick.getOutput() < -50)
  {
    stepperHoming = STEERING;
  }

  // Move and home the selected stepper
  switch (stepperHoming)
  {
  case BRAKE:
    if (setHomeSwitch.getOutput())
    {
      brakeStepper.setCurrentPosition(0);
    }
    brakeStepper.runSpeedPercentage(gasJoystick.getOutput() / 10);
    steeringStepper.runSpeedPercentage(0);
    break;

  case STEERING:
    if (setHomeSwitch.getOutput())
    {
      steeringStepper.setCurrentPosition(0);
    }
    steeringStepper.runSpeedPercentage(gasJoystick.getOutput() / 10);
    brakeStepper.runSpeedPercentage(0);
    break;

  default:
    break;
  }
}

/**
 * Updates the RC inputs with new values
 */
void updateFilters()
{
  gasJoystick.updateFilter(getPpmValue(gasJoystick.inputChannel));
  steeringJoystick.updateFilter(getPpmValue(steeringJoystick.inputChannel));
  homingModeSwitch.updateFilter(getPpmValue(homingModeSwitch.inputChannel));
  setHomeSwitch.updateFilter(getPpmValue(setHomeSwitch.inputChannel));
  towSwitch.updateFilter(getPpmValue(towSwitch.inputChannel));
  gearSwitch.updateFilter(getPpmValue(gearSwitch.inputChannel));
}
/**
 * Gets the value of a PPM channel and bounds it to an int
 * @param channel The channel to get the value of
 * @return The value of the channel
 */
int getPpmValue(int channel)
{
  unsigned int value = ppm.latestValidChannelValue(channel, 0);
  if (value > 32767)
  {
    return 32767;
  }
  else
  {
    return value;
  }
}
