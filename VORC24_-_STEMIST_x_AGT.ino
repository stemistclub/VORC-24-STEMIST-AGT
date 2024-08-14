#include <Arduino.h>
#include "motors.h"
#include "PS2_controller.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define servo pinOut.
#define Servo_PIN 7

// Servo PWM Constants
#define MIN_SERVO 93
#define MAX_SERVO 183

// Define motor pins for driving and lifting.
#define PWM_CHANNEL_LEFT 12
#define PWM_CHANNEL_RIGHT 13
#define LIFTING_MOTOR_PIN 14

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2Controller ps2;

// Lifting control variables
int liftingSpeed = 0;
static bool liftingUpActive = false;
static bool liftingDownActive = false;

void setup() {
  Serial.begin(9600);
  Serial.println("AGT's Robot setup completed");

  pwm.begin();
  pwm.setPWMFreq(60);

  pinMode(PWM_CHANNEL_LEFT, OUTPUT);
  pinMode(PWM_CHANNEL_RIGHT, OUTPUT);
  pinMode(LIFTING_MOTOR_PIN, OUTPUT);
  pinMode(Servo_PIN, OUTPUT);

  ps2.setup(13, 11, 10, 12); // setup PS2 controller pins
}

void loop() {
  ps2.read();
  
  // Read joystick values for both X and Y axes
  int joystickX = ps2.getJoystickX();
  int joystickY = ps2.getJoystickY();
  
  // Calculate motor speeds based on joystick inputs
  int motorSpeedX = (joystickX / 128.0) * 300;
  int motorSpeedY = (joystickY / 128.0) * 300;

  // Handle movement in all four quadrants
  if (joystickX > 0 && joystickY > 0) { 
    // Quadrant 1: Both X and Y are positive, move diagonally forward-right
    int combinedSpeed = sqrt(pow(motorSpeedX, 2) + pow(motorSpeedY, 2));
    analogWrite(PWM_CHANNEL_LEFT, combinedSpeed);
    analogWrite(PWM_CHANNEL_RIGHT, combinedSpeed);
  } else if (joystickX < 0 && joystickY > 0) {
    // Quadrant 2: X is negative, Y is positive, move diagonally forward-left
    int combinedSpeed = sqrt(pow(motorSpeedX, 2) + pow(motorSpeedY, 2)); 
    analogWrite(PWM_CHANNEL_LEFT, -combinedSpeed);
    analogWrite(PWM_CHANNEL_RIGHT, combinedSpeed);
  } else if (joystickX < 0 && joystickY < 0) {
    // Quadrant 3: Both X and Y are negative, move diagonally backward-left
    int combinedSpeed = sqrt(pow(motorSpeedX, 2) + pow(motorSpeedY, 2)); 
    analogWrite(PWM_CHANNEL_LEFT, -combinedSpeed);
    analogWrite(PWM_CHANNEL_RIGHT, -combinedSpeed);
  } else if (joystickX > 0 && joystickY < 0) {
    // Quadrant 4: X is positive, Y is negative, move diagonally backward-right
    int combinedSpeed = sqrt(pow(motorSpeedX, 2) + pow(motorSpeedY, 2)); 
    analogWrite(PWM_CHANNEL_LEFT, combinedSpeed);
    analogWrite(PWM_CHANNEL_RIGHT, -combinedSpeed);
  } else {
    // Center: No movement
    analogWrite(PWM_CHANNEL_LEFT, 0);
    analogWrite(PWM_CHANNEL_RIGHT, 0);
  }

  // Linear Lifting control with toggle behavior
  if (ps2.buttonPressed(PSB_TRIANGLE)) {
    liftingUpActive = !liftingUpActive; // Toggle lifting up state
    if (liftingUpActive) {
        liftingSpeed = 300; // Start lifting up
        liftingDownActive = false; // Stop any ongoing lowering
    } else {
        liftingSpeed = 0; // Stop lifting
    }
  } else if (ps2.buttonPressed(PSB_CROSS)) {
    liftingDownActive = !liftingDownActive; // Toggle lowering state
    if (liftingDownActive) {
        liftingSpeed = -300; // Start lowering
        liftingUpActive = false; // Stop any ongoing lifting
    } else {
        liftingSpeed = 0; // Stop lowering
    }
  }

  // Apply the speed to the lifting motor
  analogWrite(LIFTING_MOTOR_PIN, liftingSpeed);

  // Servo control for opening storage door
  if (ps2.buttonPressed(PSB_R2)) {
    static bool servoOpen = false;
    servoOpen = !servoOpen;
    pwm.setPWM(Servo_PIN, 0, servoOpen ? MAX_SERVO : MIN_SERVO);
  }

  // Intake motor control
  if (ps2.buttonPressed(PSB_CIRCLE)) {
    static bool intakeRunning = false;
    intakeRunning = !intakeRunning;
    digitalWrite(LIFTING_MOTOR_PIN, intakeRunning ? HIGH : LOW);
  }

  delay(50);
}
