/*
  Hand Gesture Controlled Robotic Manipulator - ESP32 Servo Controller
  ----------------------------------------------------------------------
  Receives "theta1,theta2,theta3\n" over serial from the Python
  hand-tracking script (python/hand_tracker.py) and drives three
  hobby servos accordingly:
      theta1 -> base yaw       (servo1, pin BASE_SERVO_PIN)
      theta2 -> shoulder pitch (servo2, pin SHOULDER_SERVO_PIN)
      theta3 -> gripper open/close (servo3, pin GRIPPER_SERVO_PIN)

  Baud rate MUST match the --baud argument used by hand_tracker.py
  (115200 by default).

  Author: Group 3, Dept. of Mechatronics Engineering, KUET
*/

#include <Servo.h>

constexpr int BASE_SERVO_PIN = 3;
constexpr int SHOULDER_SERVO_PIN = 5;
constexpr int GRIPPER_SERVO_PIN = 6;
constexpr long BAUD_RATE = 115200;

Servo baseServo;
Servo shoulderServo;
Servo gripperServo;

int angleBase = 90;
int angleShoulder = 90;
int angleGripper = 0;

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);

  baseServo.attach(BASE_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);

  baseServo.write(angleBase);
  shoulderServo.write(angleShoulder);
  gripperServo.write(angleGripper);

  Serial.println("Servo controller ready");
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      parseAndDrive(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += inChar;
    }
  }
}

void parseAndDrive(const String &data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  if (firstComma <= 0 || secondComma <= firstComma) {
    return;  // malformed packet, ignore
  }

  int theta1 = data.substring(0, firstComma).toInt();
  int theta2 = data.substring(firstComma + 1, secondComma).toInt();
  int theta3 = data.substring(secondComma + 1).toInt();

  theta1 = constrain(theta1, 0, 180);
  theta2 = constrain(theta2, 0, 180);
  theta3 = constrain(theta3, 0, 120);

  angleBase = 180 - theta1;  // mirror to match the camera's flipped view
  angleShoulder = theta2;
  angleGripper = theta3;

  baseServo.write(angleBase);
  shoulderServo.write(angleShoulder);
  gripperServo.write(angleGripper);

  Serial.print("Servos -> base: ");
  Serial.print(angleBase);
  Serial.print(", shoulder: ");
  Serial.print(angleShoulder);
  Serial.print(", gripper: ");
  Serial.println(angleGripper);
}
