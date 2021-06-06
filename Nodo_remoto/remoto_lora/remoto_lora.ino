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
const int csPin = 18;          // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 35;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
char var[255];
//String incoming = "";
char msg[255];



void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(3000);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  
  if (millis() - lastSendTime > interval) {
    String message(var);
    //sendMessage("3517549970&9-5-2021&hola pa, te falta mucho");
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = 30000;    // 2-3 seconds
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());

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

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  //updateSerial(incoming);
//  if (incomingLength != incoming.length()) {   // check length for error
  //  Serial.println("error: message length does not match length");
   // return;                             // skip rest of function
  //}

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  delay(5000);
  incoming.toCharArray(msg,255);
  mySerial.write(msg);
  Serial.println("Mensaje enviado a NODO GSM");
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
