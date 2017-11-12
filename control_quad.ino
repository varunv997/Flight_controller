//Headers
#include <Servo.h>

//Global Variables
unsigned long timer[5];
byte last_channel[4];
int input[4];
int error[4];
Servo m1, m2, m3, m4;

void motorRPM();

void setup() {
  // put your setup code here, to run once:
  // Setting up interrupt pins
  PCICR |= (1 << PCIE0); // To enable interrupts by specifying in Interrupt control register
  PCMSK0 |= (1 << PCINT0); // Mask register informed that pin 8 (PCINT0) will cause interrupt
  PCMSK0 |= (1 << PCINT1); // Mask register informed that pin 9 (PCINT1) will cause interrupt
  PCMSK0 |= (1 << PCINT2); // Mask register informed that pin 10 (PCINT2) will cause interrupt
  PCMSK0 |= (1 << PCINT3); // Mask register informed that pin 11 (PCINT3) will cause interrupt
  m1.attach(2); // Motor-1
  m2.attach(3); // Motor-2
  m3.attach(4); // Motor-3
  m4.attach(5); // Motor-4
  Serial.begin(9600);
  //m1.writeMicroseconds(1780);
  //delay(4000);
  //m1.writeMicroseconds(1148);
  }

void loop() {
  // put your main code here, to run repeatedly:
  print();
  motorRPM();
}

ISR(PCINT0_vect) { //Define Interrupt service routine
  timer[0] = micros(); //A time stamp is made as an interrupt is made
  
  //channel 1---------------------------------
  if (last_channel[0] == 0 && PINB & B00000001) { //check whether the channel transition is from low to high (similar to digitalRead())
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if (last_channel[0] == 1 && !(PINB & B00000001)) { //check whether the channel transition is from high to low (similar to digitalRead())
    last_channel[0] = 0;
    input[0] = timer[0] - timer[1];
    error[0] = 1506-input[0]; // error in roll
  }

  //channel 2---------------------------------
  if (last_channel[1] == 0 && PINB & B00000010) { 
    last_channel[1] = 1;
    timer[2] = timer[0];
  }
  else if (last_channel[1] == 1 && !(PINB & B00000010)) {
    last_channel[1] = 0;
    input[1] = timer[0] - timer[2];
    error[1] = 1512-input[1]; // error in pitch
  }

  //channel 3---------------------------------
  if (last_channel[2] == 0 && PINB & B0000100) { 
    last_channel[2] = 1;
    timer[3] = timer[0];
  }
  else if (last_channel[2] == 1 && !(PINB & B00000100)) {
    last_channel[2] = 0;
    input[2] = timer[0] - timer[3];
  }

  //channel 4---------------------------------
  if (last_channel[3] == 0 && PINB & B00001000) {
    last_channel[3] = 1;
    timer[4] = timer[0];
  }
  else if (last_channel[3] == 1 && !(PINB & B00001000)) {
    last_channel[3] = 0;
    input[3] = timer[0] - timer[4];
    error[3] = 1504-input[3]; // error in yaw
  }
}

void print() {
  Serial.print(input[0]);
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.print(input[3]);
  Serial.println(" ");
}

void motorRPM() {
  // correction in pwms
  m1.writeMicroseconds(input[2]+error[0]-error[1]-error[3]);
  m2.writeMicroseconds(input[2]-error[0]+error[1]-error[3]);
  m3.writeMicroseconds(input[2]-error[0]-error[1]+error[3]);
  m4.writeMicroseconds(input[2]+error[0]+error[1]+error[3]);
} 

