#include <TouchScreen.h>

#include <Adafruit_MotorShield.h>
#include <MotorDriver.h>
MotorDriver m;

long duration1;                                                           //duration of ultrasonic pulse
long duration2;
long duration3;
int distanceCm1;                                                          //distance in cm
int distanceCm2;
int distanceCm3;

void turnRight() {
  m.motor(1, FORWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, FORWARD, 0);  
  delay(2000);
  m.motor(1, BACKWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, BACKWARD, 200);
}

void turnLeft() {
  m.motor(1, FORWARD, 0);
  m.motor(2, FORWARD, 0); 
  m.motor(3, FORWARD, 0);
  m.motor(4, FORWARD, 0);  
  delay(2000);
  m.motor(1, FORWARD, 200);
  m.motor(2, BACKWARD, 200); 
  m.motor(3, BACKWARD, 200);
  m.motor(4, FORWARD, 200);
}

void goForward() {
  m.motor(1, FORWARD, 200);
  m.motor(2, FORWARD, 200);
  m.motor(3, FORWARD, 200);
  m.motor(4, FORWARD, 200);  
}

void goBackward() {
  m.motor(1, BACKWARD, 200);
  m.motor(2, BACKWARD, 200);
  m.motor(3, BACKWARD, 200);
  m.motor(4, BACKWARD, 200);  
}

void setup() {
  pinMode(A1, OUTPUT);                                                    //Analog pin A1 connected to TRIG
  pinMode(A0, INPUT);                                                     //Analog pin A0 connected to ECHO
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, INPUT);
}

void loop()
{
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  duration1 = pulseIn(A0, HIGH);                                          //check time elasped in receiving back the pulse on ECHO
  delay(200);
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);                                                 //give a pulse of 10us on TRIG
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  duration2 = pulseIn(A2, HIGH);
  delay(200);
  digitalWrite(A5, LOW);
  delayMicroseconds(2);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A5, LOW);
  duration3 = pulseIn(A4, HIGH);
  delay(200);
  m.motor(1, FORWARD, 200);
  m.motor(2, FORWARD, 200); 
  m.motor(3, FORWARD, 200);
  m.motor(4, FORWARD, 200);
  distanceCm1 = duration1 * 0.034 / 2;                                    //convert to distance in cm
  distanceCm2 = duration2 * 0.034 / 2;
  distanceCm3 = duration3 * 0.034 / 2;
  if(distanceCm1 <= 20)                                                   //if distance less than 20cm display on monitor
  {
      if(distanceCm2 <= 20 && distanceCm3 <= 20) {
          goBackward();
      }
      else if(distanceCm3 <= 20) {
          turnLeft();
      }
      else turnRight();
  }
  else {
      goForward();
  }
}
