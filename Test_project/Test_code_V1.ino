// Obstacle avoidance robot (L298N + 3 Ultrasonic Sensors)
// Updated: Swapped turning directions + reduced speed

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

// Behavior settings
const int forwardSpeed = 90;           // reduced from 200
const int turnSpeed = 80;              // slower turning
const int thresholdFront = 20;         // cm
const int thresholdSide = 5;           // cm
const unsigned long sensorDelay = 175; // ms between loops

// PID settings (for side-following while moving forward)
float pid_kp = 1.2;  // proportional gain (tweak as needed)
float pid_ki = 0.02; // integral gain
float pid_kd = 0.1;  // derivative gain
float pid_integral = 0.0;
float pid_lastError = 0.0;
unsigned long pid_lastTime = 0;
const float desiredSideDistance = 15.0; // desired distance from left wall (cm)

// Read distance (cm)
long readDistanceCM(int trigPin, int echoPin, unsigned long timeout = 30000UL)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, timeout);
  if (duration == 0)
    return -1;
  return duration / 58L;
}

// Motor helpers
void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forwardMotors(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Drive two motors with independent signed speeds
// leftSpeed/rightSpeed range: -255..255 (negative = reverse)
void setLeftMotor(int leftSpeed)
{
  if (leftSpeed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  }
}

void setRightMotor(int rightSpeed)
{
  if (rightSpeed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, 255));
  }
}

// Drive with independent left/right speeds
void driveMotors(int leftSpeed, int rightSpeed)
{
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void reverseMotors(int speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// 🔁 Swapped directions here
// Corrected: turn in place that matches function names
// Turn left in place: left wheel reverse, right wheel forward
void turnLeftInPlace(int speed)
{
  digitalWrite(IN1, LOW); // left motor reverse
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, constrain(speed, 0, 255));
  analogWrite(ENB, constrain(speed, 0, 255));
}

// Turn right in place: left wheel forward, right wheel reverse
void turnRightInPlace(int speed)
{
  digitalWrite(IN1, HIGH); // left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); // right motor reverse
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, constrain(speed, 0, 255));
  analogWrite(ENB, constrain(speed, 0, 255));
}

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);

  Serial.begin(9600);
  stopMotors();
  delay(100);
}

void loop()
{
  long distF = readDistanceCM(trigF, echoF);
  long distL = readDistanceCM(trigL, echoL);
  long distR = readDistanceCM(trigR, echoR);

  Serial.print("F: ");
  Serial.print(distF);
  Serial.print("  L: ");
  Serial.print(distL);
  Serial.print("  R: ");
  Serial.println(distR);

  if (distF == -1 || distF > thresholdFront)
  {
    // Use PID side-following while moving forward when front is clear
    // Prefer left sensor if available; if not, try right sensor mirrored
    float currentSide = NAN;
    int useLeft = 0;
    if (distL != -1)
    {
      currentSide = (float)distL;
      useLeft = 1;
    }
    else if (distR != -1)
    {
      currentSide = (float)distR;
      useLeft = 0;
    }

    if (!isnan(currentSide))
    {
      // Compute error relative to desired side distance
      float error = desiredSideDistance - currentSide;
      unsigned long now = millis();
      float dt = 0.0;
      if (pid_lastTime == 0)
      {
        dt = 0.02; // small default dt for first run
      }
      else
      {
        dt = (now - pid_lastTime) / 1000.0;
        if (dt <= 0)
          dt = 0.0001;
      }

      pid_integral += error * dt;
      float derivative = (error - pid_lastError) / dt;
      float output = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative;

      // If using right sensor instead of left, invert the sign of steering
      if (!useLeft)
        output = -output;

      // output is steering: positive -> steer to increase right motor speed
      int base = forwardSpeed;
      int leftSpeed = (int)constrain(base - output, -255, 255);
      int rightSpeed = (int)constrain(base + output, -255, 255);
      driveMotors(leftSpeed, rightSpeed);

      pid_lastError = error;
      pid_lastTime = now;
    }
    else
    {
      // No reliable side sensors: fallback to straight forward
      forwardMotors(forwardSpeed);
    }
  }
  else
  {
    stopMotors();
    delay(80);

    bool leftClear = (distL == -1) || (distL > thresholdSide);
    bool rightClear = (distR == -1) || (distR > thresholdSide);

    if (leftClear && !rightClear)
    {
      turnLeftInPlace(turnSpeed); // swapped → actually turns right
      delay(350);
      stopMotors();
      delay(50);
    }
    else if (rightClear && !leftClear)
    {
      turnRightInPlace(turnSpeed); // swapped → actually turns left
      delay(350);
      stopMotors();
      delay(50);
    }
    else if (leftClear && rightClear)
    {
      if (distL > distR)
      {
        turnLeftInPlace(turnSpeed);
        delay(300);
      }
      else
      {
        turnRightInPlace(turnSpeed);
        delay(300);
      }
      stopMotors();
      delay(50);
    }
    else
    {
      reverseMotors(forwardSpeed);
      delay(300);
      stopMotors();
      delay(100);
      turnRightInPlace(turnSpeed); // swapped
      delay(400);
      stopMotors();
      delay(50);
    }
  }

  delay(sensorDelay);
}
