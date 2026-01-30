#include <Arduino.h>

// 0 for no logs, 1 for some, and 2 for all
#define DEBUG 0

const float target_time = 50; // seconds
const float grid_size = 0.5; // meters
// instructions for the robot, scaled by square size
// +x is right, +y is forward, relative to robot start position
const int num_instructions = 16;
// {x, y}
const float instructions[num_instructions][2] = {
  {0, 0.5},
  {1, 0},
  {0, 1},
  {1, 0},
  {0, 1},
  {-1, 0},
  {0, 1},
  {0, -1},
  {-1, 0},
  {0, -1},
  {-1, 0},
  {1, 0},
  {0, 1},
  {-1, 0},
  {0, 1},
  {1, 0}
};
// backup instructions ran when pressing the button closer to the front
const int num_backup_instructions = 5;
const float backup_instructions[num_backup_instructions][2] = {
  {0, 1.5},
  {1, 0},
  {0, -1},
  {-1, 0},
  {0, 1}
};

void setup() {
  #if DEBUG >= 1
    Serial.begin(9600);
    Serial.println("\nRunning Program");
  #endif

  // intialize stepper motors
  initializeSteppers(target_time, grid_size, num_instructions, instructions);

  // and run code depending on which button is pressed
  bool run_backup_instructions = false;
  // wait until a push button is pressed
  const int button_input_pin = A0;
  const int button_output_pin = A2;
  pinMode(button_input_pin, INPUT_PULLUP);
  pinMode(button_output_pin, OUTPUT);
  const int backup_button_input_pin = A3;
  const int backup_button_output_pin = A5;
  pinMode(backup_button_input_pin, INPUT_PULLUP);
  pinMode(backup_button_output_pin, OUTPUT);
  digitalWrite(button_output_pin, LOW);
  #if DEBUG >= 1
    Serial.println("  - Waiting for start button press");
  #endif
  // loop until a start button is pressed
  while (true) {
    if (digitalRead(button_input_pin) == LOW) {
      run_backup_instructions = false;
      break;
    }
    if (digitalRead(backup_button_input_pin) == LOW) {
      run_backup_instructions = true;
      break;
    }
  }
  // wait a second so the person isn't touching the robot when it begins moving
  delay(1000);

  // follow instructions depending on which button was pressed
  if (run_backup_instructions) {
    #if DEBUG >= 1
      Serial.println("    - Following backup instructions");
    #endif
    // update speed with given instructions
    for (int i = 0; i < 10; i ++) updateSpeed(target_time, grid_size, num_backup_instructions, backup_instructions);
    // follow all the instructions
    followInstructions(grid_size, num_backup_instructions, backup_instructions);
  }
  // otherwise run normal instructions
  else {
    #if DEBUG >= 1
      Serial.println("    - Following primary instructions");
    #endif
    // update speed with given instructions
    for (int i = 0; i < 10; i ++) updateSpeed(target_time, grid_size, num_instructions, instructions);
    // follow all the instructions
    followInstructions(grid_size, num_instructions, instructions);
  }
}

void loop() { }



// Ideally, this would be in another file, but these are all the helper functions
#include <math.h>
#include <AccelStepper.h>
#include "AccelMultiStepper.h"

// Values based on cars (everything is metric)
const float metersPerRotation = 0.2443; // meters the car travels per rotation of wheels
const float degreesPerTurn = 164.25; // degrees the wheels need to rotate to turn the car 90 degrees
// So, the measuring point of the car is from the front bottom tip of a dowel, but the point of rotation is a little bit behind
// We first offset the car to the rotation point, then at the end offset back to the dowel
const float distanceToRotationPoint = 0.03429;

// Motor Specs
const int spr = 200; // Steps per revolution
const int microsteps = 1; // Stepsize (1 for full steps, 2 for half steps, 4 for quarter steps, etc)
const uint8_t MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};

// Providing parameters and pin numbers for motors
// A4988 right_stepper(spr, 2, 3, 4, 5, 6, 7);
// A4988 left_stepper(spr, 8, 9, 10, 11, 12, 13);
AccelStepper right_stepper(AccelStepper::DRIVER, 3, 2);
AccelStepper left_stepper(AccelStepper::DRIVER, 9, 8);
// Allows running both motors at the same time
// this essentailly sets the ratio of velocity to acceleration
AccelMultiStepper steppers(200.0f, 200.0f);

void initializeSteppers(float target_time, float grid_size, int num_instructions, float instructions[][2]) {
  // add enable ping to steppers and disable them
  right_stepper.setEnablePin(4);
  left_stepper.setEnablePin(10);
  right_stepper.disableOutputs();
  left_stepper.disableOutputs();
  // add steppers to multistepper
  // MUST ADD LEFT_STEPPER FIRST
  steppers.addStepper(left_stepper);
  steppers.addStepper(right_stepper);

  // set microstepping of steppers
  setMicrostep(microsteps, 5, 6, 7);
  setMicrostep(microsteps, 11, 12, 13);
}
// sets micro stepping given the ms1, ms2, and ms3 pins
void setMicrostep(int microsteps, int ms1, int ms2, int ms3) {
  uint8_t mask = MS_TABLE[microsteps - 1];
  pinMode(ms1, OUTPUT);
  pinMode(ms2, OUTPUT);
  pinMode(ms3, OUTPUT);
  digitalWrite(ms3, mask & 4);
  digitalWrite(ms2, mask & 2);
  digitalWrite(ms1, mask & 1);
}
// updates max speed and acceleration of AccelMultiStepper to match target time
float updateSpeed(float target_time, float grid_size, int num_instructions, float instructions[][2]) {
  #if DEBUG >= 1
    Serial.println("  - Calculating RPM");
  #endif

  float total_time = 0; // total rotation of the motors for all the instructions
  total_time += steppers.getTime(getForwardRotationAmount(distanceToRotationPoint) / 360 * spr);
  for (int i = 0; i < num_instructions; i ++) {

    #if DEBUG >= 2
      Serial.println("    - Instruction " + String(i));
    #endif

    if (i == 0) {
      float previous_instruction[2] = {0, 1};
      total_time += getInstructionTime(grid_size, previous_instruction, instructions[i]);
    }
    else total_time += getInstructionTime(grid_size, instructions[i - 1], instructions[i]);
  }

  total_time += steppers.getTime(getForwardRotationAmount(distanceToRotationPoint) / 360 * spr);
  total_time *= 0.9637; // calculated time appears to be a little over

  #if DEBUG >= 1
    Serial.println("    - Calculation Max Speed (steps/sec): " + String(steppers.maxSpeed()));
    Serial.println("    - Calculation Acceleration (steps/sec^2): " + String(steppers.acceleration()));
    Serial.println("    - Calculated Time (sec): " + String(total_time));
  #endif

  // update steppers speed and acceleration appropriately
  steppers.setMaxSpeed(steppers.maxSpeed() * total_time / target_time);
  steppers.setAcceleration(steppers.acceleration() * total_time / target_time);

  #if DEBUG >= 1
    Serial.println("    - New Max Speed (steps/sec): " + String(steppers.maxSpeed()));
    Serial.println("    - New Acceleration (steps/sec^2): " + String(steppers.acceleration()));
    Serial.println("    - Target Time (sec): " + String(target_time));
    Serial.println("    - NOTE: Aim for a max speed below 350 steps/sec");
  #endif
}
// returns the total rotation amount of following an instruction
float getInstructionTime(float grid_size, float previous_instruction[2], float current_instruction[2]) {
  float total_time = 0;

  // get direction of previous_instruction, in degrees clockwise relative to +x axis
  double initial_angle = -atan2(previous_instruction[1], previous_instruction[0]) * 180/PI;
  double final_angle = -atan2(current_instruction[1], current_instruction[0]) * 180/PI;
  // get rotation amount of turn
  // NOTE: rotation amoutns > 180 are turned the counter clockwise instead
  double rotation_amount = double(int(final_angle - initial_angle + 360) % 360);
  rotation_amount = (rotation_amount > 180)? rotation_amount - 360 : rotation_amount;
  total_time += steppers.getTime(getTurnRotationAmount(rotation_amount) / 360 * spr);

  #if DEBUG >= 2
    Serial.println("      - Time for turning vehicle: " + String(total_time));
  #endif

  // get forward rotation amount
  float magnitude = sqrt(pow(current_instruction[0], 2) + pow(current_instruction[1], 2));
  total_time += steppers.getTime(getForwardRotationAmount(grid_size * magnitude) / 360 * spr);

  #if DEBUG >= 2
    Serial.println("      - Time for turning AND moving vehicle: " + String(total_time));
  #endif

  return total_time;
}

void followInstructions(float grid_size, int num_instructions, float instructions[][2]) {
  #if DEBUG >= 1
    Serial.println("  - Following Instructions");
    Serial.println("    - Start millis: " + String(millis()));
  #endif

  left_stepper.enableOutputs();
  right_stepper.enableOutputs();

  #if DEBUG >= 1
    Serial.println("    - Moving to Rotation Point");
  #endif
  moveForward(distanceToRotationPoint);

  // start following actual instructions
  for (int i = 0; i < num_instructions; i ++) {
    #if DEBUG >= 1
      Serial.println("    - Following Instruction " + String(i));
    #endif

    if (i == 0) {
      float previous_instruction[2] = {0, 1};
      followInstruction(grid_size, previous_instruction, instructions[i]);
    }
    else followInstruction(grid_size, instructions[i - 1], instructions[i]);
  }

  #if DEBUG >= 1
    Serial.println("    - Moving to Measuring Point");
  #endif
  moveForward(-distanceToRotationPoint);

  right_stepper.disableOutputs();
  left_stepper.disableOutputs();

  #if DEBUG >= 1
    Serial.println("    - Finished Instructions");
    Serial.println("    - End millis: " + String(millis()));
  #endif
}
void followInstruction(float grid_size, float previous_instruction[2], float current_instruction[2]) {
  // get direction of previous_instruction, in degrees clockwise relative to +x axis
  double initial_angle = -atan2(previous_instruction[1], previous_instruction[0]) * 180/PI;
  double final_angle = -atan2(current_instruction[1], current_instruction[0]) * 180/PI;
  // make angles > 180 turn counter clockwise instead
  double rotation_amount = double(int(final_angle - initial_angle + 360) % 360);
  rotation_amount = (rotation_amount > 180)? rotation_amount - 360 : rotation_amount;
  // turn to final_angle
  #if DEBUG >= 2
    Serial.println("      - Turning " + String(rotation_amount) + " degrees clockwise");
  #endif
  turn(rotation_amount);

  // move forward the amount stated in current_instruction
  float magnitude = sqrt(pow(current_instruction[0], 2) + pow(current_instruction[1], 2));
  #if DEBUG >= 2
    Serial.println("      - Moving Forwards " + String(grid_size * magnitude) + " meters");
  #endif
  moveForward(grid_size * magnitude);
}
// positive amount is forwards
void moveForward(float amount) {
  float rotation_amount = getForwardRotationAmount(amount);
  // convert rotation_amounts to steps
  long rotation_amounts[] = {-rotation_amount / 360 * spr, rotation_amount / 360 * spr};
  steppers.moveTo(rotation_amounts);
  steppers.runToPosition();
}
// returns the rotation for the right motor, may or may not be negative
float getForwardRotationAmount(float amount) {
  return amount / metersPerRotation * 360;
}
// positive degrees rotates the car clockwise
void turn(float degrees) {
  float rotation_amount = getTurnRotationAmount(degrees);
  // convert rotation_amounts to steps
  long rotation_amounts[] = {rotation_amount / 360 * spr, rotation_amount / 360 * spr};
  steppers.moveTo(rotation_amounts);
  steppers.runToPosition();
}
// returns rotation amount for both motors, also might be negative
float getTurnRotationAmount(float degrees) {
  return -1 * degrees/90 * degreesPerTurn;
}
