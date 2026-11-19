/*
  Line-Following Robot with Obstacle Avoidance
  Author: Prabvir Grewal

  Follows a black-tape line on the floor using two IR sensors. When the
  ultrasonic sensor sees an obstacle on the line, the robot stops, turns the
  servo to look left and right, drives around the obstacle on the clearer
  side, then searches for the line and continues its route.

  Hardware
    - Arduino Uno
    - L293D motor shield (Adafruit Motor Shield v1 compatible)
    - 4x DC gear motors (M1..M4 on the shield)
    - 2x IR line sensors (digital output)
    - HC-SR04 ultrasonic sensor, mounted on a servo
    - SG90 micro servo on the shield's SERVO_1 header (pin 10)

  The motor shield uses digital pins 3, 4, 5, 6, 7, 8, 11 and 12, so the
  sensors are wired to the analog header (A0-A3), used here as digital pins.

  Library: "Adafruit Motor Shield library" (AFMotor) by Adafruit, v1.0.x,
  installed from the Arduino Library Manager. Servo.h ships with the IDE.
*/

#include <AFMotor.h>
#include <Servo.h>

// ---------------- Pins ----------------
const uint8_t IR_LEFT_PIN  = A0;
const uint8_t IR_RIGHT_PIN = A1;
const uint8_t TRIG_PIN     = A2;
const uint8_t ECHO_PIN     = A3;
const uint8_t SERVO_PIN    = 10;   // SERVO_1 header on the shield

// Most IR modules output HIGH over black tape and LOW over a light floor.
// If yours is the other way round, change this to LOW.
const int ON_LINE = HIGH;

// ---------------- Tuning ----------------
const uint8_t  CRUISE_SPEED      = 150;  // 0-255 PWM
const uint8_t  TURN_SPEED        = 170;
const uint8_t  AVOID_SPEED       = 160;
const uint16_t OBSTACLE_CM       = 15;   // stop if something is closer than this
const uint16_t AVOID_TURN_MS     = 450;  // time for roughly a 45 degree pivot
const uint16_t AVOID_SIDE_MS     = 700;  // drive sideways past the obstacle
const uint16_t AVOID_PASS_MS     = 900;  // drive alongside the obstacle
const uint16_t LINE_SEARCH_MS    = 2500; // give up searching after this long

// Servo angles for the ultrasonic "head"
const uint8_t LOOK_LEFT   = 160;
const uint8_t LOOK_CENTER = 90;
const uint8_t LOOK_RIGHT  = 20;

// ---------------- Hardware objects ----------------
// M1 = front left, M2 = rear left, M3 = rear right, M4 = front right.
// Swap these if your wiring differs.
AF_DCMotor frontLeft(1);
AF_DCMotor rearLeft(2);
AF_DCMotor rearRight(3);
AF_DCMotor frontRight(4);
Servo head;

enum State { FOLLOW_LINE, AVOID_OBSTACLE, FIND_LINE };
State state = FOLLOW_LINE;

// Which way the robot last saw the line, used to recover if both sensors lose it.
enum Side { NONE, LEFT_SIDE, RIGHT_SIDE };
Side lastSeen = NONE;

// ---------------- Motor helpers ----------------
// Drive each side of the robot. Positive = forward, negative = backward.
void setSide(AF_DCMotor &a, AF_DCMotor &b, int speed) {
  uint8_t dir = speed >= 0 ? FORWARD : BACKWARD;
  uint8_t pwm = constrain(abs(speed), 0, 255);
  a.setSpeed(pwm); b.setSpeed(pwm);
  a.run(dir);      b.run(dir);
}

void drive(int left, int right) {
  setSide(frontLeft, rearLeft, left);
  setSide(frontRight, rearRight, right);
}

void stopMotors() {
  frontLeft.run(RELEASE); rearLeft.run(RELEASE);
  frontRight.run(RELEASE); rearRight.run(RELEASE);
}

// ---------------- Sensor helpers ----------------
bool leftOnLine()  { return digitalRead(IR_LEFT_PIN)  == ON_LINE; }
bool rightOnLine() { return digitalRead(IR_RIGHT_PIN) == ON_LINE; }

// Returns distance in cm, or 999 if nothing echoes back within ~4 m.
uint16_t readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (us == 0) return 999;
  return us / 58;  // speed of sound: ~58 us per cm, round trip
}

// Median of 3 readings filters out single bad echoes.
uint16_t distanceFiltered() {
  uint16_t a = readDistanceCm(); delay(10);
  uint16_t b = readDistanceCm(); delay(10);
  uint16_t c = readDistanceCm();
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

uint16_t lookAt(uint8_t angle) {
  head.write(angle);
  delay(350);  // let the servo settle before measuring
  return distanceFiltered();
}

// ---------------- Behaviours ----------------
void followLine() {
  bool l = leftOnLine();
  bool r = rightOnLine();

  if (!l && !r) {
    // Line is between the sensors: go straight.
    drive(CRUISE_SPEED, CRUISE_SPEED);
  } else if (l && !r) {
    // Line drifted under the left sensor: steer left.
    drive(-TURN_SPEED / 2, TURN_SPEED);
    lastSeen = LEFT_SIDE;
  } else if (!l && r) {
    drive(TURN_SPEED, -TURN_SPEED / 2);
    lastSeen = RIGHT_SIDE;
  } else {
    // Both on black: a crossing or a thick section. Keep going straight.
    drive(CRUISE_SPEED, CRUISE_SPEED);
  }
}

void avoidObstacle() {
  stopMotors();
  uint16_t left  = lookAt(LOOK_LEFT);
  uint16_t right = lookAt(LOOK_RIGHT);
  head.write(LOOK_CENTER);

  Serial.print(F("Obstacle! left=")); Serial.print(left);
  Serial.print(F(" cm, right=")); Serial.print(right); Serial.println(F(" cm"));

  // Go around on whichever side has more room.
  int s = (right >= left) ? 1 : -1;   // +1 = pass on the right, -1 = left

  drive(AVOID_SPEED * s, -AVOID_SPEED * s);  delay(AVOID_TURN_MS);   // turn away
  drive(AVOID_SPEED, AVOID_SPEED);           delay(AVOID_SIDE_MS);   // move off the line
  drive(-AVOID_SPEED * s, AVOID_SPEED * s);  delay(AVOID_TURN_MS);   // straighten up
  drive(AVOID_SPEED, AVOID_SPEED);           delay(AVOID_PASS_MS);   // pass the obstacle
  drive(-AVOID_SPEED * s, AVOID_SPEED * s);  delay(AVOID_TURN_MS);   // angle back toward the line
  stopMotors();

  lastSeen = (s == 1) ? LEFT_SIDE : RIGHT_SIDE;  // line is now on the opposite side
  state = FIND_LINE;
}

void findLine() {
  unsigned long start = millis();
  drive(CRUISE_SPEED, CRUISE_SPEED);
  while (millis() - start < LINE_SEARCH_MS) {
    if (leftOnLine() || rightOnLine()) {
      // Turn back onto the line in the direction we came from.
      if (lastSeen == LEFT_SIDE) drive(-TURN_SPEED, TURN_SPEED);
      else                       drive(TURN_SPEED, -TURN_SPEED);
      delay(AVOID_TURN_MS / 2);
      state = FOLLOW_LINE;
      Serial.println(F("Line found, resuming route"));
      return;
    }
  }
  // Could not find the line: stop safely instead of wandering off.
  stopMotors();
  Serial.println(F("Line lost, stopped"));
  while (true) { delay(1000); }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(9600);
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  head.attach(SERVO_PIN);
  head.write(LOOK_CENTER);
  stopMotors();
  delay(1000);  // time to put the robot on the line
}

void loop() {
  switch (state) {
    case FOLLOW_LINE: {
      // Ping only every 60 ms: a ping can block for up to 25 ms, and the
      // line sensors need to be read as often as possible.
      static unsigned long lastPing = 0;
      if (millis() - lastPing >= 60) {
        lastPing = millis();
        if (readDistanceCm() < OBSTACLE_CM) {
          state = AVOID_OBSTACLE;
          break;
        }
      }
      followLine();
      break;
    }
    case AVOID_OBSTACLE:
      avoidObstacle();
      break;
    case FIND_LINE:
      findLine();
      break;
  }
}
