// Obstacle avoidance robot (L298N + 3 Ultrasonic Sensors)
// Updated: Swapped turning directions + reduced speed

#define ENA 6
#define ENB 5
#define IN1 2
#define IN2 4
#define IN3 3
#define IN4 7

#define trigF 8
#define echoF A5
#define trigL 10
#define echoL 11
#define trigR 9
#define echoR A4


// --- Optimized parameters for maze environment ---
const int forwardSpeed = 85;     // balanced speed for smooth travel
const int turnSpeed = 75;        // softer turning
const int thresholdFront = 10;    // stop before 18 cm
const int thresholdSide = 6;      // maintain 6 cm from side walls
const unsigned long sensorDelay = 60; // stabilize sensor sampling


// Read distance (cm)
long readDistanceCM(int trigPin, int echoPin, unsigned long timeout = 2000UL) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, timeout);
  if (duration == 0) return -1;
  return duration / 58L;
}

// Motor helpers
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forwardMotors(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void reverseMotors(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// 🔁 Swapped directions here
void turnLeftInPlace(int speed) {
  // Now rotates RIGHT instead of LEFT
  digitalWrite(IN1, LOW);  // left motor forward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);   // right motor reverse
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnRightInPlace(int speed) {
  // Now rotates LEFT instead of RIGHT
  digitalWrite(IN1, HIGH);   // left motor reverse
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // right motor forward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void setup() {
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

void loop() {
  long distF = readDistanceCM(trigF, echoF);
  long distL = readDistanceCM(trigL, echoL);
  long distR = readDistanceCM(trigR, echoR);

  Serial.print("F: "); Serial.print(distF);
  Serial.print("  L: "); Serial.print(distL);
  Serial.print("  R: "); Serial.println(distR);

  if (distF == -1 || distF > thresholdFront) {
    forwardMotors(forwardSpeed);
  } else {
    stopMotors();
    delay(80);

    bool leftClear = (distL == -1) || (distL > thresholdSide);
    bool rightClear = (distR == -1) || (distR > thresholdSide);

    if (leftClear && !rightClear) {
      turnLeftInPlace(turnSpeed);  // swapped → actually turns right
      delay(350);
      stopMotors();
      delay(50);
    } else if (rightClear && !leftClear) {
      turnRightInPlace(turnSpeed); // swapped → actually turns left
      delay(350);
      stopMotors();
      delay(50);
    } else if (leftClear && rightClear) {
      if (distL > distR) {
        turnLeftInPlace(turnSpeed);
        delay(300);
      } else {
        turnRightInPlace(turnSpeed);
        delay(300);
      }
      stopMotors();
      delay(50);
    } else {
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
