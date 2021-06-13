#include <HardwareSerial.h>

HardwareSerial MySerial(2);

String var = "";
//Create software serial object to communicate with SIM800L
// SoftwareSerial mySerial(16, 17); //
void updateSerial();
void setup()
{      
  Serial.begin(9600);
  while (!Serial);
  //Begin serial communication with Arduino and SIM800L
  MySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(3000);

}

void loop()
{
  updateSerial();
}

void updateSerial()
{
  delay(500);
  MySerial.write("hola desde C++");//Forward what Serial received to Software Serial Port
  Serial.print("enviado...");
  
  while(MySerial.available())
  {
      Serial.print(char(MySerial.read()));//Forward what Software Serial received to Serial Port
  }
}
