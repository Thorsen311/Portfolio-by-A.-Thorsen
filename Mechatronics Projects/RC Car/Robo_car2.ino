/*
   Arduino Robot Car Wireless Control using the HC-05 Bluetooth and custom-build Android app

             == SLAVE DEVICE - Arduino robot car ==

   by Dejan Nedelkovski, www.HowToMechatronics.com
*/
int enA = 9;
int in1 = 4;
int in2 = 5;
int enB = 10;
int in3 = 6;
int in4 = 7;
int SpeedA = 0;
int SpeedB = 0;
String Direction;

void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  Direction = ""; // Reset the Direction string at the beginning of each loop iteration

  while (Serial.available()){
    delay(50);
    char c = Serial.read();
    Direction += c;
  }

  Direction.trim(); // Remove leading and trailing whitespace characters

  if(Direction.length() > 0){
    Serial.println(Direction);
    
    if (Direction == "F") {
      // Move Forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      SpeedA = 150;
      SpeedB = 150;
    } else if (Direction == "B") {
      // Move Backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      SpeedA = 150;
      SpeedB = 150;
    } else if (Direction == "L") {
      // Left Turn
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      SpeedA = 120;
      SpeedB = 0;
    } else if (Direction == "R") {
      // Right Turn
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      SpeedA = 0;
      SpeedB = 120;
    } else if (Direction == "S") {
      // STOP
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      SpeedA = 0;
      SpeedB = 0;
    }
    
    analogWrite(enA, SpeedA); // Send PWM signal to motor A
    analogWrite(enB, SpeedB); // Send PWM signal to motor B
    Serial.println(SpeedA);
  }
}
