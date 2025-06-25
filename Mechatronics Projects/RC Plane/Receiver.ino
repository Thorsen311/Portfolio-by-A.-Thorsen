//  4 Channel Receiver | 4 Kanal Alıcı
//  PWM output on pins D2, D3, D4, D5 (Çıkış pinleri)

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;

Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;

struct Signal {
byte throttle;      
byte pitch;
byte roll;
byte yaw;
};

Signal data;

const uint64_t pipeIn = 0xE9E8F0F0E1LL;
RF24 radio(7, 8); 

void ResetData()
{
// Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
// The middle position for Potenciometers. (254/2=127) | Potansiyometreler için orta konum
data.throttle = 127; // Motor Stop | Motor Kapalı
data.pitch = 127;  // Center | Merkez
data.roll = 127;   // Center | Merkez
data.yaw = 127;   // Center | Merkez
}

void setup()
{
  Serial.begin(9600);
  //Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
  ch1.attach(2); //Throttle er på D2, så ch1 skal være throttle
  ch2.attach(3); //Set forfra er det den højre servo
  ch3.attach(4); //Bagerste servo
  ch4.attach(5); //Set forfra er det den venstre servo

  //Configure the NRF24 module
  ResetData();
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  
  radio.startListening(); //start the radio comunication for receiver | Alıcı olarak sinyal iletişimi başlatılıyor
}

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(Signal));
lastRecvTime = millis();   // receive the data | data alınıyor
}
}

void loop()
{
recvData();
unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
ResetData(); // Signal lost.. Reset data | Sinyal kayıpsa data resetleniyor
}
// Map incoming data to the full range of servo angles (0 to 180 degrees)
//ch_width_1 = map(data.throttle, 0, 255, 0, 180);
ch_width_1 = map(data.throttle, 0, 255, 1100, 1700);     // pin D2 (PWM signal), motoren aktiveres ved omkring PWM 1500, du kan øge hastigheden ved at øge det sidste tal fra 1700-2000
ch_width_2 = map(data.pitch,    0, 255, 0, 180);        // Styrer alle 3 servos med højre joystick frem-tilbage
ch_width_3 = map(data.roll,     0, 255, 0, 180);        // Styres 2 vinge-servos med højre joystick venstre-højre
ch_width_4 = map(data.yaw,      0, 255, 0, 180);       // Ubrugt

int pitch_value = ch_width_2;  // Map pitch to servo range
int roll_value = ch_width_3;   // Map roll to servo range

// Apply pitch control if roll is within 80-100 range
if (roll_value > 70 && roll_value < 110) {
    ch2.write(map(pitch_value, 0, 180, 20, 100));  // Set pitch for ch2, set forfra er det højre servo
    ch3.write(map(pitch_value, 0, 180, 180, 40));  // Set pitch for ch3, bagerste servo
    ch4.write(map(pitch_value, 0, 180, 100, 20));  // Set pitch for ch4, set forfra er det venstre servo, de sidste to tal er vendt rundt fordi den skal være inverted
}

// Apply roll control if pitch is within 80-100 range
if (pitch_value > 70 && pitch_value < 110) {
    ch2.write(map(roll_value,0,180,0,100));  // Set roll for ch3 (inversed), højre servo set forfra
    ch4.write(map(roll_value,0,180,20,100));    // Set roll for ch4, venstre servo set forfra
}

// Set the throttle and yaw positions
ch1.write(ch_width_1);  // Set throttle (PWM signal on pin D2)


Serial.print("Throttle: ");
Serial.print(ch_width_1);
Serial.print(", Roll: ");
Serial.print(ch_width_3);
Serial.print(", Pitch: ");
Serial.print(ch_width_2);
Serial.print(", Yaw: ");
Serial.println(ch_width_4);
}
