/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss.
*/

#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDL mag;

char report[80];
bool IN_A,IN_B;
int PinA = 10;
int PinB = 11;
float MagneticOffset = 0;
float head = 0;
float prev_head = 0;
float prev_mx = 0;
float prev_my = 0;
float prev_mz = 0;

float mx = 0;
float my = 0;
float mz = 0;


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
  for (int i=0;i<=50;i++){
    MagneticOffset += HeadingData();
    delay(10);
  }
  MagneticOffset = MagneticOffset/50;
  
}

void loop()
{
    float angle = HeadingData()-MagneticOffset;
    Serial.print("  angle  ");
    Serial.println(angle);
     // motor (255 ,'ccw');

}

int HeadingData(){
    mag.read();
    float a = 0.2;
    float b = 1;
    mx = mag.m.x;
    my = mag.m.y;
    mz = mag.m.z;

    mx = mx*a+(a-1)*prev_mx;
    my = my*a+(a-1)*prev_my;
    mz = mz*a+(a-1)*prev_mz;

    prev_mx = mx;
    prev_my = my;
    prev_mz = mz;
    
    head=180*atan2(mx,my)/PI;
    Serial.print("Hx ");
    Serial.print(mx);
    Serial.print("  Hy ");    
    Serial.print(my);
    
    Serial.print("  Raw Head  ");
    Serial.print(head);
    if(mx < 0 && my <0) head = 360+head;
    else if(mx < 0 && my >0) head = 360+head; 
    else if(my == 0 && mx <0) head = 180;
    else if(my == 0 && mx >0) head = 0;

    head = head*b+(b-1)*prev_head;
    prev_head = head;
    Serial.print("  Head  ");
    Serial.print(head);
//    delay(200);
    return head;
}
//void motor(char direc){
//  if(direc == 'ccw'){
//    IN_A = HIGH;
//    IN_B = LOW;
//  }
//  if(direc == 'cw'){
//    IN_A = LOW;
//    IN_B = HIGH;
//  }
//  digitalWrite(PinA , IN_A);
//  digitalWrite(PinB , IN_B);  
//}

