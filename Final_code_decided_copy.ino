#include <AFMotor.h>
#include <Servo.h>

#define TRIG_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 100 // Maximum distance (in cm) to detect obstacles
#define STOP_DISTANCE 10 // Distance (in cm) to stop the vehicle

String command;

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  myservo.write(90);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  delay(2000);
  
  // Trigger pulse for ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the pulse from the echo pin
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check for obstacle
  if (distance < STOP_DISTANCE) {
    // If an object is detected within STOP_DISTANCE, stop the motors
    stop();
    Serial.println("Stopped due to obstacle");
  } else {
    // If no obstacle detected, execute command if available
    if (Serial.available()) {
      command = Serial.readString();
      Serial.println(command);

      // Execute command based on received input
      if (command == "*move forward#") {
        forward(); 
        Serial.println("Moving forward");
      } else if (command == "*move backward#") {
        backward();
        Serial.println("Moving backward");
      } else if (command == "*turn left#") {
        left();
        Serial.println("Turning left");
      } else if (command == "*turn right#") {
        right();
        Serial.println("Turning right");
      } else if (command == "*stop#") {
        stop();
        Serial.println("Stopped");
      }
    }
  }
}

void forward() {
  
  myservo.write(90);
  // If no obstacle, move forward
  motor1.setSpeed(255);
  motor1.run(FORWARD);
  motor2.setSpeed(255);
  motor2.run(FORWARD);
  motor3.setSpeed(255);
  motor3.run(FORWARD);
  motor4.setSpeed(255);
  motor4.run(FORWARD);

  // Delay to ensure movement
  delay(1000);

  // Check for obstacle again after movement
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  if (distance < STOP_DISTANCE) {
    stop(); // If an obstacle is detected after movement, stop the motors
    Serial.println("Stopped due to obstacle after forward movement");
  }
}

// Similar modifications for other movement functions (backward, left, right)...
void right() {
  myservo.write(145);
  delay(1500);
  myservo.write(0);
  delay(1000);
  myservo.write(90);
  motor1.setSpeed(255);
  motor1.run(BACKWARD);
  motor2.setSpeed(255);
  motor2.run(BACKWARD);
  motor3.setSpeed(255);
  motor3.run(FORWARD);
  motor4.setSpeed(255);
  motor4.run(FORWARD);
  delay(50);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  if (distance < STOP_DISTANCE) {
    stop(); // If an obstacle is detected after movement, stop the motors
    Serial.println("Stopped due to obstacle after forward movement");
  }
}

void left() {
  myservo.write(60);
  delay(1500);
  myservo.write(180);
  delay(1000);
  myservo.write(90);
  motor1.setSpeed(255);
  motor1.run(FORWARD);
  motor2.setSpeed(255);
  motor2.run(FORWARD);
  motor3.setSpeed(255);
  motor3.run(BACKWARD);
  motor4.setSpeed(255);
  motor4.run(BACKWARD);
  delay(50);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  if (distance < STOP_DISTANCE) {
    stop(); // If an obstacle is detected after movement, stop the motors
    Serial.println("Stopped due to obstacle after forward movement");
  }
}

void backward() {
  myservo.write(360);
  delay(1500);
  myservo.write(0);
  motor1.setSpeed(255);
  motor1.run(BACKWARD);
  motor2.setSpeed(255);
  motor2.run(BACKWARD);
  motor3.setSpeed(255);
  motor3.run(BACKWARD);
  motor4.setSpeed(255);
  motor4.run(BACKWARD);
  delay(5000);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  if (distance < STOP_DISTANCE) {
    stop(); // If an obstacle is detected after movement, stop the motors
    Serial.println("Stopped due to obstacle after forward movement");
  }
}


  


void stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
