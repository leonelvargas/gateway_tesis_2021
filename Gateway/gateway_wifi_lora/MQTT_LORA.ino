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
#include <WiFi.h>
#include <PubSubClient.h>

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
int rotation = 0;
char var[255];
bool telegram = false;
String incoming = "";

TaskHandle_t Task1;
const char* ssid     = "CLAROFIBRA1"; //Nombre y contraseña del WIFI a utilizar
const char* password = "JADE120203";

const char *mqtt_server = "gatewaymultip.ml"; //Dominio
const int mqtt_port = 1883; //Puerto designado por el Broker para comunicacion TCP
const char *mqtt_user = "web_client_acom"; //usuario y contraseña valido de la base de datos para el ingreso
const char *mqtt_pass = "123456789"; //Esto se debe modificar para cada dispositivo y tiene que estar en la base de datos admmin_basegaten
int wifi_on = 0;

WiFiClient espClient;
PubSubClient client(espClient); //Se indica el tipo de comunicacion dispositivo a Broker

char msg[255];


//*****************************
//*** DECLARACION FUNCIONES ***
//*****************************
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
int setup_wifi();



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

  randomSeed(micros()); //semilla para la creacion de n° aleatorios
  if (setup_wifi()){
    wifi_on = 1; //inicializacion del WIFI
  }
  client.setServer(mqtt_server, mqtt_port); //Conexion con el Broker
  client.setCallback(callback);
  xTaskCreatePinnedToCore(
    codeForTask1, /* Task function. */
    "Task_1",  /* name of task. */
    6000,     /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &Task1,     /* Task handle to keep track of created task */
    0);       /* Core */
}

void loop() {
   if (millis() - lastSendTime > interval) {
    String message(var);
    //sendMessage("3517549970&9-5-2021&hola pa, te falta mucho");
    if (telegram){
      Serial.println(rotation);
      sendMessage(incoming);
      Serial.println("Sending " + incoming);
      rotation++;
      }
    else {
      sendMessage(message);
      Serial.println("Sending " + message);
      }
    if (rotation == 4){
      telegram = false;
      rotation = 0;
      }
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

//*****************************
//***    CONEXION WIFI      ***
//*****************************
int setup_wifi(){
  int i=0;
  delay(10);
  // Nos conectamos a nuestra red Wifi
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  for(i=0;i<4;i++){
    WiFi.begin(ssid, password);
    if (WiFi.status() != WL_CONNECTED) { //se observa la siguiente salida
      delay(30000);             //mientras se espera la conexion exitosa: ...............
      Serial.print("Intento n°:");
      Serial.println(i);
    } 
    else{
      Serial.println("");
      Serial.println("Conectado a red WiFi!");
      Serial.println("Dirección IP: "); //Se muestra la ip asignada al dispositivo por el router
      Serial.println(WiFi.localIP());
      return 0;
    }
  }

  if (i=4){
    return 1;
    Serial.println("");
    Serial.println("No se pudo conectar");
    }

}


//*****************************
//***      MENSAJERIA       ***
//*****************************
void callback(char* topic, byte* payload, unsigned int length){ //La funcion recibe el topico, el array de bytes y su dimension
  Serial.print("Mensaje recibido desde -> "); //se usa print en vez de println para
  Serial.print(topic);            //mostrar una palabra tras otra
  Serial.println("");
  for (int i = 0; i < length; i++) { //Se obtiene el mensaje recibido (array de bytes)
    incoming += (char)payload[i]; // y se pasa a string para manipulacion
  }
  incoming.trim(); //Al transformar un array de bytes a string existen espacion en blancos al final de la palabra
  Serial.println("Mensaje -> " + incoming); //suprimidos por dicha funcion
  telegram = true;
  Serial.println("Sending " + incoming);

}

void reconnect() {

  while (!client.connected()) {
    Serial.print("Intentando conexión Mqtt...");
    // Creamos un cliente ID aleatorio con el fin de no superponerse, en el caso de multiples conexiones del mismo usuario
    String clientId = "esp32_";
    clientId += String(random(0xffff), HEX);
    // Intentamos conectar
    if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
      Serial.println("Conectado!");
      // Nos suscribimos a los topicos 
      client.subscribe("123456789/insert_msjtaken_query");
    } else {
      Serial.print("falló :( con error -> ");
      Serial.print(client.state()); //Muestra el estado actual de la conexion con el broker
      Serial.println(" Intentamos de nuevo en 5 segundos");

      delay(5000); //Y luego se vuelve a ejecutar el while, si todavia no se conecto con el Broker
    }
  }
}


//*****************************
//******     Core 2     *******
//*****************************

void codeForTask1(void *parameter)
{
  for (;;)
  { 
    Serial.println("holaaa");
    delay(10000);
    if (!client.connected() && wifi_on == 0) {
      reconnect(); //Llamada a la funcion, en el caso que se caiga la conexion MQTT con el Broker
    }
    if (wifi_on == 0){
      client.loop(); //Llamada constante
    }
  }
}
