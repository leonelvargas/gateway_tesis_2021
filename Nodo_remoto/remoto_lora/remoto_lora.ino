/*
  LoRa Duplex communication
  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.
  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an
  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <HardwareSerial.h>
HardwareSerial mySerial(2);
#define ss 5
#define rst 14
#define dio0 2

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
char var[255];
String incoming = "";
volatile bool received = false; // Flag set by callback to perform read process in main loop
volatile int incomingPacketSize;
char msg[255];



void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(3000);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(ss, rst, dio0);
    
  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  
  LoRa.onReceive(onReceive);
  Serial.println("LoRa init succeeded.");
}

void loop() {
  
  if (millis() - lastSendTime > interval) {
    String message(var);
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = 30000;    // 2-3 seconds
  }

  // parse for a packet, and call onReceive with the result:
  LoRa.receive();
  delay (5000);
  if(received){
   readMessage();
  }
  received = false;
  //updateSerial();
  //delay(5000);
  //Serial.print("ACA TENDRIA QUE ESTAR INCOMING:");
  //Serial.print("enviando al nodo GSM:");
  //mySerial.write("3517549970&9-5-2021&asdqwerty");//Forward what Serial received to Software Serial Port
  //mySerial.write(msg);  
  updateSerial();

  
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void readMessage() {
  // received a packet
  incoming = "";
  Serial.print("Received packet '");

  // read packet
  for (int i = 0; i < incomingPacketSize; i++) {
    incoming += (char)LoRa.read();
  }
  Serial.print(incoming);
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
  incoming.toCharArray(msg,255);
  mySerial.write(msg);
  Serial.println("Mensaje enviado a NODO GSM");
}

void onReceive(int packetSize) {
  received = true;
  incomingPacketSize = packetSize;
}

void updateSerial()
{
  //Serial.print("ACA TENDRIA QUE ESTAR INCOMING:");
  //Serial.print("enviando al nodo GSM:");
  //mySerial.write("3517549970&9-5-2021&asdqwerty");//Forward what Serial received to Software Serial Port
  //mySerial.write(msg);  

  while(mySerial.available())
  {
    delay(5000);
    for(int i=0;i<255;i++){
      var[i] = char(mySerial.read());//Forward what Software Serial received to Serial Port
      Serial.print(var[i]);
      }
  }
}
