#include <Servo.h>

// Constants
const int FULL_ROTATION = 180;
const int MAX_PWM = 255;
const int MIN_PWM = 0;

// Drivetrain
int enAd = 2;
int in1d = 22;
int in2d = 23;
int enBd = 3;
int in3d = 26;
int in4d = 27;

char driveDir;

// Lean FB, LR
int enAl = 4;
int in1l = 32;
int in2l = 33;
int enBl = 5;
int in3l = 36;
int in4l = 37;

float linearPosFB = 0;
long lastActFB;
int lastDirFB = 0;
int desiredLeanFBAngle = 0;
int angleFB = 0;

float linearPosLR = 0;
long lastActLR;
int lastDirLR = 0;
int desiredLeanLRAngle = 0;
int angleLR = 0;

// Arms and Neck
Servo neckPan;
Servo neckTilt;
Servo rightShoulderTwist;
Servo rightShoulder;
Servo rightElbow;
Servo rightWrist;
Servo leftShoulderTwist;
Servo leftShoulder;
Servo leftElbow;
Servo leftWrist;

float neckPanPos;
float neckTiltPos;
float rightShoulderTwistPos;
float rightShoulderPos;
float rightElbowPos;
float rightWristPos;
float leftShoulderTwistPos;
float leftShoulderPos;
float leftElbowPos;
float leftWristPos;

float neckPanTar;
float neckTiltTar;
float rightShoulderTwistTar;
float rightShoulderTar;
float rightElbowTar;
float rightWristTar;
float leftShoulderTwistTar;
float leftShoulderTar;
float leftElbowTar;
float leftWristTar;


unsigned long lastServoUpdate = millis();
float deltaTime = 0;
float servoSpeed = 0.3;

// Gripper
int in3g = 40;
int in4g = 41;

void setup() {
  pinMode(enAd, OUTPUT);
  pinMode(in1d, OUTPUT);
  pinMode(in2d, OUTPUT);
  pinMode(enBd, OUTPUT);
  pinMode(in3d, OUTPUT);
  pinMode(in4d, OUTPUT);

  pinMode(enAl, OUTPUT);
  pinMode(in1l, OUTPUT);
  pinMode(in2l, OUTPUT);
  pinMode(enBl, OUTPUT);
  pinMode(in3l, OUTPUT);
  pinMode(in4l, OUTPUT);

  pinMode(in3g, OUTPUT);
  pinMode(in4g, OUTPUT);

  rightShoulderTwist.attach(6);
  rightShoulder.attach(7);
  rightElbow.attach(8);
  rightWrist.attach(9);
  leftShoulderTwist.attach(10);
  leftShoulder.attach(11);
  leftElbow.attach(12);
  leftWrist.attach(13);
  neckPan.attach(44);
  neckTilt.attach(45);

  neckPanTar = 0;
  neckTiltTar = 0;
  rightShoulderTwistTar = 0;
  rightShoulderTar = -90;
  rightElbowTar = 0;
  rightWristTar = 0;
  leftShoulderTwistTar = 0;
  leftShoulderTar = -90;
  leftElbowTar = 0;
  leftWristTar = 0;

  neckPanPos = neckPanTar - 2;
  neckTiltPos = neckTiltTar - 2;
  rightShoulderTwistPos = rightShoulderTwistTar - 2;
  rightShoulderPos = rightShoulderTar - 2;
  rightElbowPos = rightElbowTar - 2;
  rightWristPos = rightWristTar - 2;
  leftShoulderTwistPos = leftShoulderTwistTar - 2;
  leftShoulderPos = leftShoulderTar - 2;
  leftElbowPos = leftElbowTar - 2;
  leftWristPos = leftWristTar - 2;

  moveServos();

  Serial.begin(9600);
}

void loop() {
  updateActuators();
  moveServos();
  lean();
}

void moveServos() {
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastServoUpdate) / 1000.0;  // Convert to seconds
  lastServoUpdate = currentTime;

  moveRightShoulder(deltaTime);
  moveRightShoulderTwist(deltaTime);
  moveRightElbow(deltaTime);
  moveRightWrist(deltaTime);
  moveLeftShoulder(deltaTime);
  moveLeftShoulderTwist(deltaTime);
  moveLeftElbow(deltaTime);
  moveLeftWrist(deltaTime);
  moveNeckPan(deltaTime);
  moveNeckTilt(deltaTime);
}

void updateActuators() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    if (data == "inMotion") {
      Serial.println(inMotion() ? "true" : "false");
    } else if (data == "clench") {
      clench();
    } else if (data == "release") {
      release();
    } else if (data == "zeroFB") {
      linearPosFB = 0;
      desiredLeanFBAngle = 0;
    } else if (data == "zeroLR") {
      linearPosLR = 0;
      desiredLeanLRAngle = 0;
    } else if (data.substring(0, 2) == "LF") {
      desiredLeanFBAngle = data.substring(2, 6).toInt();
    } else if (data.substring(0, 2) == "LL") {
      desiredLeanLRAngle = data.substring(2, 6).toInt();
    } else if (data.charAt(0) == 'D') {
      String leftSpeedStr = data.substring(1, 3);
      String rightSpeedStr = data.substring(3, 5);
      int leftSpeed = (leftSpeedStr.toInt() - 50) * 2;
      int rightSpeed = (rightSpeedStr.toInt() - 50) * 2;
      drive(leftSpeed, rightSpeed);
    } else {
      String angle = data.substring(2, 6);

      if (data.charAt(0) == 'L') {
        if (data.charAt(1) == 'W') {
          leftWristTar = angle.toInt();
        } else if (data.charAt(1) == 'S') {
          leftShoulderTar = angle.toInt();
        } else if (data.charAt(1) == 'T') {
          leftShoulderTwistTar = angle.toInt();
        } else if (data.charAt(1) == 'E') {
          leftElbowTar = angle.toInt();
        }
      } else if (data.charAt(0) == 'R') {
        if (data.charAt(1) == 'W') {
          rightWristTar = angle.toInt();
        } else if (data.charAt(1) == 'S') {
          rightShoulderTar = angle.toInt();
        } else if (data.charAt(1) == 'T') {
          rightShoulderTwistTar = angle.toInt();
        } else if (data.charAt(1) == 'E') {
          rightElbowTar = angle.toInt();
        }
      } else if (data.charAt(0) == 'N') {
        if (data.charAt(1) == 'P') {
          neckPanTar = angle.toInt();
        } else if (data.charAt(1) == 'T') {
          neckTiltTar = angle.toInt();
        }
      }
    }
    Serial.flush();
  }
}

bool inMotion() {
  return (
    abs(leftWristPos - leftWristTar) > 1 || abs(leftShoulderPos - leftShoulderTar) > 1 || abs(leftElbowPos - leftElbowTar) > 1 || abs(leftShoulderTwistPos - leftShoulderTwistTar) > 1 || abs(rightWristPos - rightWristTar) > 1 || abs(rightShoulderPos - rightShoulderTar) > 1 || abs(rightElbowPos - rightElbowTar) > 1 || abs(rightShoulderTwistPos - rightShoulderTwistTar) > 1 || abs(neckPanPos - neckPanTar) > 1 || abs(neckTiltPos - neckTiltTar) > 1 || abs(angleFB - desiredLeanFBAngle) > 1 || abs(angleLR - desiredLeanLRAngle) > 1);
}


//////////////////////////////////////////////////// DRIVING ////////////////////////////////////////////////////

void drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  int leftDirection = (leftSpeed >= 0) ? LOW : HIGH;
  int rightDirection = (rightSpeed >= 0) ? LOW : HIGH;

  int leftPWM = map(abs(leftSpeed), 0, 100, 0, 255);
  int rightPWM = map(abs(rightSpeed), 0, 100, 0, 255);

  analogWrite(enAd, leftPWM);
  digitalWrite(in1d, leftDirection);
  digitalWrite(in2d, !leftDirection);

  analogWrite(enBd, rightPWM);
  digitalWrite(in3d, rightDirection);
  digitalWrite(in4d, !rightDirection);
}


//////////////////////////////////////////////////// LEANING ////////////////////////////////////////////////////

void lean() {
  linearPosFB += (lastDirFB * (10.0 / 1000) * (millis() - lastActFB));
  lastActFB = millis();
  angleFB = linearPosFB * 1;  // Approximated

  if ((linearPosFB < 20) && (angleFB < desiredLeanFBAngle)) {
    analogWrite(enBl, MAX_PWM);
    digitalWrite(in3l, LOW);
    digitalWrite(in4l, HIGH);
    lastDirFB = 1;
  } else if ((linearPosFB > -20) && (angleFB > desiredLeanFBAngle)) {
    analogWrite(enBl, MAX_PWM);
    digitalWrite(in3l, HIGH);
    digitalWrite(in4l, LOW);
    lastDirFB = -1;
  } else {
    analogWrite(enBl, MIN_PWM);
    lastDirFB = 0;
  }

  linearPosLR += (lastDirLR * (10.0 / 1000) * (millis() - lastActLR));
  lastActLR = millis();
  angleLR = linearPosLR * 0.64;  // Approximated

  if ((linearPosLR < 10) && (angleLR < desiredLeanLRAngle)) {
    analogWrite(enAl, MAX_PWM);
    digitalWrite(in1l, LOW);
    digitalWrite(in2l, HIGH);
    lastDirLR = 1;
  } else if ((linearPosLR > -10) && (angleLR > desiredLeanLRAngle)) {
    analogWrite(enAl, MAX_PWM);
    digitalWrite(in1l, HIGH);
    digitalWrite(in2l, LOW);
    lastDirLR = -1;
  } else {
    analogWrite(enAl, MIN_PWM);
    lastDirLR = 0;
  }
}

//////////////////////////////////////////////////// GRIPPER ////////////////////////////////////////////////////

void clench() {
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    digitalWrite(in3g, HIGH);
    digitalWrite(in4g, LOW);
  }
  digitalWrite(in3g, LOW);
  digitalWrite(in4g, LOW);
}

void release() {
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    digitalWrite(in3g, LOW);
    digitalWrite(in4g, HIGH);
  }
  digitalWrite(in3g, LOW);
  digitalWrite(in4g, LOW);
}

//////////////////////////////////////////////////// SERVOS ////////////////////////////////////////////////////

void moveRightShoulder(float deltaTime) {
  rightShoulderTar = constrain(rightShoulderTar, -FULL_ROTATION, 0);
  if (abs(rightShoulderTar - rightShoulderPos) > 1) {
    int dir = (rightShoulderTar - rightShoulderPos) / abs(rightShoulderTar - rightShoulderPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    rightShoulderPos += dir * movement;
    int pwmVal = map(rightShoulderPos, -FULL_ROTATION, 0, 2500, 500);
    rightShoulder.writeMicroseconds(pwmVal);
  }
}

void moveLeftShoulder(float deltaTime) {
  leftShoulderTar = constrain(leftShoulderTar, -FULL_ROTATION, 0);
  if (abs(leftShoulderTar - leftShoulderPos) > 1) {
    int dir = (leftShoulderTar - leftShoulderPos) / abs(leftShoulderTar - leftShoulderPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    leftShoulderPos += dir * movement;
    int pwmVal = map(leftShoulderPos, -FULL_ROTATION, 0, 500, 2500);
    leftShoulder.writeMicroseconds(pwmVal);
  }
}

void moveRightShoulderTwist(float deltaTime) {
  rightShoulderTwistTar = constrain(rightShoulderTwistTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(rightShoulderTwistTar - rightShoulderTwistPos) > 1) {
    int dir = (rightShoulderTwistTar - rightShoulderTwistPos) / abs(rightShoulderTwistTar - rightShoulderTwistPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    rightShoulderTwistPos += dir * movement;
    int pwmVal = map(rightShoulderTwistPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 2140, 860);
    rightShoulderTwist.writeMicroseconds(pwmVal);
  }
}

void moveLeftShoulderTwist(float deltaTime) {
  leftShoulderTwistTar = constrain(leftShoulderTwistTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(leftShoulderTwistTar - leftShoulderTwistPos) > 1) {
    int dir = (leftShoulderTwistTar - leftShoulderTwistPos) / abs(leftShoulderTwistTar - leftShoulderTwistPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    leftShoulderTwistPos += dir * movement;
    int pwmVal = map(leftShoulderTwistPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 860, 2140);
    leftShoulderTwist.writeMicroseconds(pwmVal);
  }
}

void moveLeftElbow(float deltaTime) {
  leftElbowTar = constrain(leftElbowTar, 0, FULL_ROTATION);
  if (abs(leftElbowTar - leftElbowPos) > 1) {
    int dir = (leftElbowTar - leftElbowPos) / abs(leftElbowTar - leftElbowPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    leftElbowPos += dir * movement;
    int pwmVal = map(leftElbowPos, 0, FULL_ROTATION, 611, 2500);
    leftElbow.writeMicroseconds(pwmVal);
  }
}

void moveRightElbow(float deltaTime) {
  rightElbowTar = constrain(rightElbowTar, 0, FULL_ROTATION);
  if (abs(rightElbowTar - rightElbowPos) > 1) {
    int dir = (rightElbowTar - rightElbowPos) / abs(rightElbowTar - rightElbowPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    rightElbowPos += dir * movement;
    int pwmVal = map(rightElbowPos, 0, FULL_ROTATION, 611, 2500);
    rightElbow.writeMicroseconds(pwmVal);
  }
}

void moveRightWrist(float deltaTime) {
  rightWristTar = constrain(rightWristTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(rightWristTar - rightWristPos) > 1) {
    int dir = (rightWristTar - rightWristPos) / abs(rightWristTar - rightWristPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    rightWristPos += dir * movement;
    int pwmVal = map(rightWristPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 860, 2140);
    rightWrist.writeMicroseconds(pwmVal);
  }
}

void moveLeftWrist(float deltaTime) {
  leftWristTar = constrain(leftWristTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(leftWristTar - leftWristPos) > 1) {
    int dir = (leftWristTar - leftWristPos) / abs(leftWristTar - leftWristPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    leftWristPos += dir * movement;
    int pwmVal = map(leftWristPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 2140, 860);
    leftWrist.writeMicroseconds(pwmVal);
  }
}

void moveNeckPan(float deltaTime) {
  neckPanTar = constrain(neckPanTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(neckPanTar - neckPanPos) > 1) {
    int dir = (neckPanTar - neckPanPos) / abs(neckPanTar - neckPanPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    neckPanPos += dir * movement;
    int pwmVal = map(neckPanPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 2140, 860);
    neckPan.writeMicroseconds(pwmVal);
  }
}

void moveNeckTilt(float deltaTime) {
  neckTiltTar = constrain(neckTiltTar, -FULL_ROTATION / 2, FULL_ROTATION / 2);
  if (abs(neckTiltTar - neckTiltPos) > 1) {
    int dir = (neckTiltTar - neckTiltPos) / abs(neckTiltTar - neckTiltPos);
    float movement = servoSpeed * deltaTime * FULL_ROTATION;
    neckTiltPos += dir * movement;
    int pwmVal = map(neckTiltPos, -FULL_ROTATION / 2, FULL_ROTATION / 2, 2140, 860);
    neckTilt.writeMicroseconds(pwmVal);
  }
}
