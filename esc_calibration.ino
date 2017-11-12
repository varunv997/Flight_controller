#include <Servo.h>



Servo s1, s2, s3, s4;

void setup() {
  // put your setup code here, to run once:
  s1.attach(6); s2.attach(9); s3.attach(10); s4.attach(11);
  s1.writeMicroseconds(1980);
  s2.writeMicroseconds(1980);
  s3.writeMicroseconds(1980);
  s4.writeMicroseconds(1980);
  delay(5000);
  s1.writeMicroseconds(1190);
  s2.writeMicroseconds(1190);
  s3.writeMicroseconds(1190);
  s4.writeMicroseconds(1190);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:


  s1.writeMicroseconds(1200);
  s2.writeMicroseconds(1200);
  s3.writeMicroseconds(1200);
  s4.writeMicroseconds(1200);
}
