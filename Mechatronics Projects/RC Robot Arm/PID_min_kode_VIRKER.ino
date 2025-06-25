// Include the library:
#include "SharpIR.h"

#include <Servo.h>

const int servoPin = 9;
const int sensorPin = A0;
#define IRPin A0
#define model 1080

int distance_cm;
SharpIR mySensor = SharpIR(IRPin, model);

Servo myServo;

// PID constants
double Kp = 2;  // Proportional constant
double Ki = 0.1;  // Integral constant
double Kd = 8.0;  // Derivative constant

// PID variables
double setpoint = 12;  // Setpoint for the sensor (midpoint)
double previousError = 0.0;
double integral = 0.0;

void setup() {
  myServo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  double sensorValue = mySensor.distance();
  double error = setpoint - sensorValue;
  
  // Proportional term
  double P = Kp * error;
  
  // Integral term
  integral += Ki * error;
  
  // Derivative term
  double derivative = Kd * (error - previousError);
  
  // PID control signal
  double pidOutput = -(P + integral + derivative);
  
  // Update servo position
  int servoPosition = constrain(135 - pidOutput, 95, 175);
  myServo.write(servoPosition);
  
  // Update previous error for the next iteration
  previousError = error;
  
  // Print values for debugging
  Serial.print("Sensor: ");
  Serial.print(sensorValue);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  PID Output: ");
  Serial.print(pidOutput);
  Serial.print("  Pos: ");
  Serial.println(servoPosition);
  delay(10);  // Adjust the delay as needed
}