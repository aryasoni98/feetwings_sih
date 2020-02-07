
#include <SoftwareSerial.h>
SoftwareSerial BTserial(10, 11); // RX | TX

const int knockSensor = A0; 
const int threshold = 100;
int count=1;


int sensorReading = 0;

void setup() {
 
BTserial.begin(9600);}

void loop() {
  sensorReading = analogRead(knockSensor);

  if (sensorReading >= threshold) {
    
    if(count>=1)
    {    BTserial.print(count);
    BTserial.print(";"); 
         count=count+1;}

  }
  delay(100);  
}
