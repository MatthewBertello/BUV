#include <PPMReader.h>

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

PPMReader ppm(config::PPM_INTERRUPT_PIN, 10); // The PPM reader
Steppers stepperHoming = BRAKE;               // The stepper that is currently being homed

// Setup the RC inputs
RcInput gasJoystick{RcInput::CENTER_JOYSTICK, config::RIGHT_STICK_UP_DOWN, &ppm};          // The joystick for the gas/brake
RcInput steeringJoystick{RcInput::CENTER_JOYSTICK, config::LEFT_STICK_LEFT_RIGHT, &ppm};   // The joystick for the steering
RcInput homingModeSwitch{RcInput::SWITCH, config::TOP_LEFT_SWITCH, &ppm};                  // The homing mode switch
RcInput setHomeSwitch{RcInput::SWITCH, config::TOP_LEFT_CENTER_SWITCH, &ppm};              // The set home switch
RcInput towSwitch{RcInput::SWITCH, config::TOP_RIGHT_SWITCH, &ppm};                        // The tow switch
RcInput gearSwitch{RcInput::THREE_POSITION_SWITCH, config::TOP_RIGHT_CENTER_SWITCH, &ppm}; // The forward/reverse Switch

// Setup the stepper motors
Stepper brakeStepper{AccelStepper::DRIVER, config::BRAKE_STEPPER_PULSE_PIN, config::BRAKE_STEPPER_DIR_PIN};          // The stepper motor for the brake
Stepper steeringStepper{AccelStepper::DRIVER, config::STEERING_STEPPER_PULSE_PIN, config::STEERING_STEPPER_DIR_PIN}; // The stepper motor for the steering wheel

//* !This code is for debugging purposes only
unsigned long lastPrint;
unsigned long printRate = 500;

/**
 * Runs once before the main loop
 */
void setup()
{

  
  //* !This code is for debugging purposes only
  Serial.begin(9600); // Start the serial port
  Serial.println("Start");
  

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

  // Setup the homing mode switch
  homingModeSwitch.minInput = config::HOMING_MODE_SWITCH_MIN_INPUT;
  homingModeSwitch.maxInput = config::HOMING_MODE_SWITCH_MAX_INPUT;

  // Setup the set home switch
  setHomeSwitch.minInput = config::SET_HOME_SWITCH_MIN_INPUT;
  setHomeSwitch.maxInput = config::SET_HOME_SWITCH_MAX_INPUT;

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
}

/**
 * The main loop
 */
void loop()
{
  
  // !This code is for debugging purposes only
  // Print current values
  if (millis() - lastPrint > printRate)
  {
    lastPrint = millis();
    for (int i = 1; i <= 10; i++)
    {
      int x = ppm.latestValidChannelValue(i, 0);
      //Serial.print(i);
      Serial.print(x);
      Serial.print(" ");
    }
    
     Serial.println();
     /**
     Serial.print("gasJoystick input: ");
     Serial.print(gasJoystick.getCurrentInput());
     Serial.print(" ");
     Serial.print("gasJoystick output: ");
     Serial.print(gasJoystick.getOutput());
     Serial.print(" ");
     Serial.print("brakeStepper position: ");
     Serial.print(brakeStepper.currentPosition());
     Serial.print(" ");
     Serial.print("brakeStepper target Position: ");
     Serial.print(brakeStepper.targetPosition());
     Serial.print(" ");
     Serial.print("accel distance: ");
     Serial.print(brakeStepper.distanceToGo());
     Serial.println();
     */
     Serial.print("ci: ");
     Serial.print(gearSwitch.getCurrentInput());
     Serial.print(" ");
     Serial.print("forward/Reverse: ");
     Serial.print(gearSwitch.getOutput());
     Serial.print(" ");
     Serial.print("towSwitch: ");
     Serial.print(towSwitch.getOutput());
     
     Serial.println();
  }
  

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

      steeringStepper.moveToInRange(steeringJoystick.getOutput());        // Set the target for the steering stepper
      digitalWrite(config::TOW_SWITCH_OUTPUT_PIN, towSwitch.getOutput()); // Set the tow switch output

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
    }

    // Run the stepper motors
    brakeStepper.runThreshold();
    steeringStepper.runThreshold();
  }
} // End of main loop

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
    if (setHomeSwitch.getOutput())
    {
      brakeStepper.setCurrentPosition(brakeStepper.currentPosition());
    }
    brakeStepper.runSpeedPercentage(gasJoystick.getOutput() / 10);
    steeringStepper.runSpeedPercentage(0);
    break;

  case STEERING:
    if (setHomeSwitch.getOutput())
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
