#include <Arduino.h>

// Motor driver pin assignments
#define ENA 6
#define ENB 5
#define IN1 4
#define IN2 2
#define IN3 3
#define IN4 7

// Ultrasonic sensor pin assignments
#define trigF 8
#define echoF A5
#define trigL 10
#define echoL 11
#define trigR 9
#define echoR A4

// Runtime/state variables
int cou = 0; // used to gate a left-turn sequence after first loop

// Tuning coefficients (used for side-following corrections)
float kl = 5; // left correction gain
float kr = 5; // right correction gain
float k = 1;  // additional multiplier used in one branch

// Simple error / P-controller variables
float error = 0;
float kp = 20; // present but not actively used in provided logic

// Default forward speeds (floats declared but loop redeclares ints later)
float forwardSpeedl = 100;
float forwardSpeedr = 100;

// Motion constants
const int turnleft = 180;
const int turnright = 180;

// Sensing thresholds (cm)
const int thresholdFront = 10;
const int thresholdSide = 20;
const unsigned long sensorDelay = 100; // not currently used but kept for clarity

// Arrays / extra state (present but unused in main logic)
long arrayf[3] = {0, 0, 0};
long arrayr[3] = {0, 0, 0};
long arrayl[3] = {0, 0, 0};
int cou1 = 0;
long distFp = 0;
long distRp = 0;
long distLp = 0;

// ---------------------------
// Helper: Read distance from an HC-SR04-style ultrasonic sensor
// Returns: distance in cm, or -1 on timeout/no echo
// Parameters:
//  - trigPin: digital pin connected to TRIG
//  - echoPin: digital pin connected to ECHO
//  - timeout: microseconds to wait for pulseIn (default 30000UL)
// ---------------------------
long readDistanceCM(int trigPin, int echoPin, unsigned long timeout = 30000UL)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, timeout);
  if (duration == 0)
    return -1;           // no echo (out of range or timeout)
  return duration / 58L; // convert microseconds to cm (approx.)
}

// ---------------------------
// Motor helper functions
// Note: These functions directly map pins to motor directions and speeds
// They intentionally preserve the original semantics and parameter order.
// ---------------------------
void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Drive forward: speedr -> motor on ENA, speedl -> motor on ENB
void forwardMotors(int speedr, int speedl)
{
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENA, speedr);
  analogWrite(ENB, speedl);
}

// Drive in reverse: speedr -> motor on ENA, speedl -> motor on ENB
void reverseMotors(int speedr, int speedl)
{
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(ENA, speedr);
  analogWrite(ENB, speedl);
}

// Turn left in place (left wheel reverse, right wheel forward)
void turnLeftInPlace(int speed)
{
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Turn right in place (right wheel reverse, left wheel forward)
void turnRightInPlace(int speed)
{
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// ---------------------------
// Arduino setup(): pin modes and initial state
// ---------------------------
void setup()
{
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);

  // Serial for debugging
  Serial.begin(9600);
  stopMotors();
  delay(100);
}

// ---------------------------
// Main loop: read sensors, decide motion
// Logical flow and numeric values preserved exactly as requested.
// ---------------------------
void loop()
{

  // Local forward speeds (these shadow the float defaults above intentionally)
  int forwardSpeedl = 200;
  int forwardSpeedr = 200;

  // Read distances from front, left, right sensors
  long distF = readDistanceCM(trigF, echoF);
  long distL = readDistanceCM(trigL, echoL);
  long distR = readDistanceCM(trigR, echoR);

  // Debug printout
  Serial.print("F: ");
  Serial.print(distF);
  Serial.print("  L: ");
  Serial.print(distL);
  Serial.print("  R: ");
  Serial.println(distR);

  // Determine whether sides are clear (treat -1 as 'no reading' => clear)
  bool leftClear = (distL == -1) || (distL > thresholdSide);
  bool rightClear = (distR == -1) || (distR > thresholdSide);

  // If left side becomes clear after initial run, perform a deliberate left-turn sequence
  if (leftClear && cou == 1)
  {
    stopMotors();
    delay(10);
    forwardMotors(100, 100);
    delay(700);
    stopMotors();
    delay(10);
    turnLeftInPlace(turnleft);
    delay(200);
    stopMotors();
    delay(20);
    forwardMotors(100, 100);
    delay(500);
    stopMotors();
    delay(20);
  }
  else
  {
    // Normal obstacle avoidance / wall-following behavior
    stopMotors();
    delay(50);

    if (distF == -1 || distF > thresholdFront)
    {
      // Path ahead is clear
      if (distF > 25)
      {
        if (!leftClear && !rightClear)
        {
          // Both sides blocked -> perform side-based correction sequence
          error = distL - 8;
          forwardMotors(forwardSpeedr + (error * kl), forwardSpeedl - (error * kl)); // -+
          delay(10);
          delay(1);
          error = distR - 5;
          forwardMotors(forwardSpeedr - (error * kr), forwardSpeedl + (error * kr)); // +-
          delay(10);
          delay(1);
        }
        else if (rightClear)
        {
          // Favor right-clear case with alternate correction
          error = distL - 8;
          forwardMotors(255 - (error * kl * k), 255 + (error * kl * k));
          delay(10);
          delay(1);
        }
      }
      else
      {
        // Slow forward if front distance is between thresholdFront and 25
        forwardMotors(90, 90);
        delay(300);
        stopMotors();
        delay(5);
      }
    }
    else if (rightClear)
    {
      // Front blocked but right side clear: perform right turn maneuver
      stopMotors();
      delay(5);
      forwardMotors(90, 90);
      delay(500);
      stopMotors();
      delay(500);
      turnRightInPlace(turnright);
      delay(240);
      stopMotors();
      delay(500);
      forwardMotors(90, 90);
      delay(500);
      stopMotors();
      delay(500);
    }
    else
    {
      // Otherwise: back up a bit and turn right in place
      stopMotors();
      delay(5);
      reverseMotors(forwardSpeedr, forwardSpeedl);
      delay(0);
      stopMotors();
      delay(50);
      // Default to turning right in place
      turnRightInPlace(turnright);
      delay(400);
      stopMotors();
      delay(500);
    }
  }

  // mark that the first loop has completed (used by the leftClear branch above)
  cou = 1;
}
