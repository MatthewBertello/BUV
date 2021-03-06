#include <AccelStepper.h>
#include <MultiStepper.h>

#include "RcInput.h"
#include "Stepper.h"
#include "config.h"

unsigned long lastInputCheck{0}; // The last time the inputs were checked
enum Steppers                    // A list of the steppers
{
  BRAKE,
  STEERING,
};
Steppers stepperHoming = BRAKE; // The stepper that is currently being homed

// Setup the RC inputs
RcInput gasJoystick{RcInput::CENTER_JOYSTICK, config::GAS_JOYSTICK_INPUT_PIN};                 // The joystick for the gas/brake
RcInput steeringJoystick{RcInput::CENTER_JOYSTICK, config::STEERING_JOYSTICK_INPUT_PIN};       // The joystick for the steering
RcInput homingModeSwitch{RcInput::SWITCH, config::TOP_LEFT_SWITCH_INPUT_PIN};                  // The homing mode switch
RcInput topLeftCenterSwitch{RcInput::SWITCH, config::TOP_LEFT_CENTER_SWITCH_INPUT_PIN};        // Unassigned switch
RcInput towSwitch{RcInput::SWITCH, config::TOP_RIGHT_SWITCH_INPUT_PIN};                        // The tow switch
RcInput gearSwitch{RcInput::THREE_POSITION_SWITCH, config::TOP_RIGHT_CENTER_SWITCH_INPUT_PIN}; // The forward/reverse Switch

// Setup the stepper motors
Stepper brakeStepper{AccelStepper::DRIVER, config::BRAKE_STEPPER_PULSE_PIN, config::BRAKE_STEPPER_DIR_PIN};          // The stepper motor for the brake
Stepper steeringStepper{AccelStepper::DRIVER, config::STEERING_STEPPER_PULSE_PIN, config::STEERING_STEPPER_DIR_PIN}; // The stepper motor for the steering wheel

/**
 * !This code is for debugging purposes only
unsigned long lastPrint;
unsigned long printRate = 5000;
*/

/**
 * Runs once before the main loop
 */
void setup()
{
  /**
   * !This code is for debugging purposes only
  Serial.begin(9600); // Start the serial port
  Serial.println("Start");
  */

  // Setup the output pins
  pinMode(config::MAIN_MOTOR_OUPTUT_PIN, OUTPUT);
  analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, 0);
  pinMode(config::TOW_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, 0);
  pinMode(config::FOOT_SWITCH_OUTPUT_PIN, OUTPUT);
  digitalWrite(config::FOOT_SWITCH_OUTPUT_PIN, 0);

  // Setup the gas joystick
  gasJoystick.minInput = config::GAS_JOYSTICK_MIN_INPUT;
  gasJoystick.maxInput = config::GAS_JOYSTICK_MAX_INPUT;
  gasJoystick.deadzone = config::GAS_JOYSTICK_DEADZONE;
  gasJoystick.centerInput = config::GAS_JOYSTICK_CENTER_VALUE;

  // Setup the steering joystick
  steeringJoystick.minInput = config::STEERING_JOYSTICK_MIN_INPUT;
  steeringJoystick.maxInput = config::STEERING_JOYSTICK_MAX_INPUT;
  steeringJoystick.deadzone = config::STEERING_JOYSTICK_DEADZONE;

  // Setup the homing mode switch
  homingModeSwitch.minInput = config::HOMING_MODE_SWITCH_MIN_INPUT;
  homingModeSwitch.maxInput = config::HOMING_MODE_SWITCH_MAX_INPUT;

  // Setup the tow switch
  towSwitch.minInput = config::TOW_SWITCH_MIN_INPUT;
  towSwitch.maxInput = config::TOW_SWITCH_MAX_INPUT;

  // Setup the gear switch
  gearSwitch.minInput = config::GEAR_SWITCH_MIN_INPUT;
  gearSwitch.maxInput = config::GEAR_SWITCH_MAX_INPUT;
  gearSwitch.centerInput = config::GEAR_SWITCH_CENTER_VALUE;

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

  // Add the interrups for the RC inputs
  attachInterrupt(digitalPinToInterrupt(gasJoystick.inputPin), gasJoystickPinStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(steeringJoystick.inputPin), steeringJoystickPinStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(homingModeSwitch.inputPin), topLeftSwitchPinStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gearSwitch.inputPin), toprightSwitchPinStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(topLeftCenterSwitch.inputPin), topLeftCenterSwitchPinStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(towSwitch.inputPin), topRightCenterSwitchPinStateChange, CHANGE);
}

/**
 * The main loop
 */
void loop()
{
  /**
   * !This code is for debugging purposes only
  // Print current values
  if(millis() - lastPrint > printRate)
  {
    lastPrint = millis();
    Serial.print("gasJoystick input: ");
    Serial.println(gasJoystick.currentInput);
    Serial.print("gasJoystick output: ");
    Serial.println(gasJoystick.getMappedOutput());
    Serial.print("brakeStepper position: ");
    Serial.println(brakeStepper.currentPosition());
    Serial.print("brakeStepper target Position: ");
    Serial.println(brakeStepper.targetPosition());
    Serial.println();
  }
  */

  if (!homingModeSwitch.getOutput() && !config::DISABLE_HOMING_MODE) // If the top left switch is off run the homing function
  {
    homingMode();
  }
  else // Otherwise run the normal operation
  {
    // if it has been long enough since the last input check get the new inputs
    if (millis() - lastInputCheck >= config::INPUT_REFRESH_RATE)
    {
      lastInputCheck = millis(); // Update the last input check time

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
        analogWrite(config::MAIN_MOTOR_OUPTUT_PIN, mathFunctions::map(abs(gasInput), 0, 100, config::MINIMUM_OUTPUT_FOR_MAIN_MOTOR_THROTTLE, 255));
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

      steeringStepper.moveToInRange(steeringJoystick.getOutput());                  // Set the target for the steering stepper
      digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, topLeftCenterSwitch.getOutput()); // Set the tow switch output
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
        digitalWrite(config::FORWARD_SWITCH_OUTPUT_PIN, HIGH);
        digitalWrite(config::REVERSE_SWITCH_OUTPUT_PIN, LOW);
      }
    }

    // Run the stepper motors
    brakeStepper.runThreshold();
    steeringStepper.runThreshold();
  }
} // End of main loop

// Create the interrupt functions for the RC inputs
void gasJoystickPinStateChange()
{
  gasJoystick.pinStateChange();
}
void steeringJoystickPinStateChange()
{
  steeringJoystick.pinStateChange();
}
void topLeftSwitchPinStateChange()
{
  homingModeSwitch.pinStateChange();
}
void toprightSwitchPinStateChange()
{
  gearSwitch.pinStateChange();
}
void topLeftCenterSwitchPinStateChange()
{
  topLeftCenterSwitch.pinStateChange();
}
void topRightCenterSwitchPinStateChange()
{
  towSwitch.pinStateChange();
}

/**
 * A function to set the home positions of the steppers
 */
void homingMode()
{
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
    if (gearSwitch.getOutput())
    {
      brakeStepper.setCurrentPosition(brakeStepper.currentPosition());
    }
    brakeStepper.runSpeedPercentage(gasJoystick.getOutput() / 10);
    steeringStepper.runSpeedPercentage(0);
    break;

  case STEERING:
    if (gearSwitch.getOutput())
    {
      steeringStepper.setCurrentPosition(steeringStepper.currentPosition());
    }
    steeringStepper.runSpeedPercentage(gasJoystick.getOutput() / 10);
    brakeStepper.runSpeedPercentage(0);
    break;

  default:
    break;
  }
}