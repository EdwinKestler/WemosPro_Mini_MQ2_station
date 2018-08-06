/*
  Remote smoke detection and emviroment node.
  Created by Edwin Kestler, May 29, 2018.
  Released into the public domain.
   MITLicense View Settings.h for more info;
  You must keep the attribution on the PCB and schematic and in the firmware files, as is
  released version 1.0 6 agosto 2018
*/
#include <Arduino.h>
// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>                                              //https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
//----------------------------------------------------------------------librerias de TIEMPO NTP
#include <TimeLibEsp.h>                                                  //TimeTracking
#include <WiFiUdp.h>                                                  //UDP packet handling for NTP request
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>                                         //Libreira de html para ESP8266
#include <DNSServer.h>                                                //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>                                              //https://github.com/tzapu/WiFiManager
//----------------------------------------------------------------------Librerias de Codigo de Lectora RFID
#include "settings.h"
//----------------------------------------------------------------------Libreria locales de clases de botron y LEDS
#include "BlinkRGB.h"
//************************************************Inicio general de librerias***********************************
#include <Wire.h>
//************************************************Librerias de Sensores Digitales*******************************
#include "Adafruit_HTU21DF.h"
//************************************************Librerias de Pantalla OLED************************************
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//---------------------------------------------------------------------------------RGB Settings
BlinkRGB Azul (D6);
BlinkRGB Verde (D7);
BlinkRGB Rojo (D8);

BlinkColor Blanco  (D6,D7,D8);
BlinkColor Purpura (D6,D4,D8);
//----------------------------------------------------------------------------------Buzzer Settings
const int beep = D5;
unsigned int count = 0;
String msg = "";
int WifiSignal;
//----------------------------------------------------------------------Variables de verificacion de fallas de capa de conexion con servicio
int failed, sent, published;                                          //Variables de conteo de envios 
//----------------------------------------------------------------------Inicio de cliente UDP
WiFiUDP udp;                                                          //Cliente UDP para WIFI
//----------------------------------------------------------------------Codigo para estblecer el protocolo de tiempo en red NTP
const int NTP_PACKET_SIZE = 48;                                       //NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                   //Buffer to hold incoming & outgoing packets
boolean NTP = false;                                                  //Bandera que establece el estado inicial del valor de NTP
//----------------------------------------------------------------------Variables del servicio de envio de datos MQTT
const char* cserver = "";
//char authMethod[] = "use-token-auth";                                 //Tipo de Autenticacion para el servicio de Bluemix (la calve es unica por cada nodo)
//char token[] = TOKEN;                                                 //Variable donde se almacena el Token provisto por el servicio (ver Settings.h)
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;             //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 
String  Smacaddrs = "00:00:00:00:00:00";
String  Sipaddrs  = "000.000.000.000";
//----------------------------------------------------------------------Declaracion de Variables Globales (procuar que sean las minimas requeridas.)
int DeviceState = 0;
unsigned long last_State_Update;                                       //Variable para llevar conteo del tiempo desde la ultima publicacion
unsigned long last_NTP_Update;                                       //Variable para llevar conteo del tiempo desde la ultima publicacion  
unsigned long last_Local_Warning;                                         //Variable para llevar conteo del tiempo desde la ultima publicacion
unsigned long time_Normal_Reset;                                       //Variable para llevar conteo del tiempo desde la ultima publicacion 
String ISO8601;                                                       //Variable para almacenar la marca del timepo (timestamp) de acuerdo al formtao ISO8601
int hora = 0;
//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                              //Variable Global que contiene la identidad del nodo (ChipID) o numero unico
//----------------------------------------------------------------------Funcion remota para administrar las actulizaciones remotas de las variables configurables desde IBMbluemix
void handleUpdate(byte* payload) {                                    //La Funcion recibe lo que obtenga Payload de la Funcion Callback que vigila el Topico de subcripcion (Subscribe TOPIC)
  StaticJsonBuffer<300> jsonBuffer;                                  //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payload);          //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
    }                                                                 //se cierra el condicional
  Serial.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}

//------------------------------------------------------------------------------------denifinir el sonido de bocina
void buzzer() 
{
  digitalWrite(beep, HIGH);
  delay(300);
  digitalWrite(beep, LOW);
  delay(100);
}

//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleResponse (byte* payloadrsp) 
{
  StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) 
  {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
  }                                                                   //se cierra el condicional
  
  Serial.println(F("handleResponse payload:"));                       //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.printTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}

//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength)
{//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
  Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje seÃ±alando sobre que topico se recibio un mensaje
  Serial.println(topic);                                              //Imprimir el Topico
  
  if (strcmp (responseTopic, topic) == 0) 
  {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
    handleResponse(payload);
    //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
  }
  
  if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.reset();                                                    //Emitir comando de reinicio para ESP8266
  }
  
  if (strcmp (updateTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    handleUpdate(payload);                                            //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
  } 
}
//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(MQTTServer, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT
//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() 
{
  if (!!!client.connected()) 
  {                                         //Verificar si el cliente se encunetra conectado al servicio
  Serial.print(F("Reconnecting MQTT client to: "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  Serial.println(MQTTServer);                                             //Imprimir la direccion del servidor a donde se esta intentado conectar 
  char charBuf[30];
  String CID (clientId + NodeID); 
  CID.toCharArray(charBuf, 30);  
  #if defined (internetS)
    while (!!!client.connect(charBuf, "user", "password")) 
    {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #else
    while (!!!client.connect(charBuf)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #endif  
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}

//----------------------------------------------------------------------Funcion de REConexion a Servicio de MQTT
void MQTTreconnect() {
  int retry = 0;
  // Loop until we're reconnected
  while (!client.connected()) {    
    Serial.print(F("Attempting MQTT connection..."));
    Blanco.CFlash();
    buzzer();
    char charBuf[30];
    String CID (clientId + NodeID);
    CID.toCharArray(charBuf, 30);  
     #if defined (internetS)
     if (client.connect(charBuf, "user", "password")) {
      Serial.println(F("connected"));
     }
     #else
     if (client.connect(charBuf)) {
      Serial.println(F("connected"));
     }
     #endif
     else {
      Purpura.CFlash();
      buzzer();
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.print(F(" try again in 3 seconds,"));
      Serial.print(F(" retry #:"));
      Serial.println(retry);
      if (retry > 10){
        ESP.restart();
        retry=0;
      }
      retry++;
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}
//************************************************Variables de Pantalla OLED************************************
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
bool New_screen_display = true;
//*************************************************Variables de Promedio de datos sensores**********************
#define NUMSAMPLES    10
float HTU_Temp =      0;
float Old_HTU_Temp =  0;
float HTU_Hum =       0;
float Old_HTU_Hum =   0;
//*************************************************VAriables de Sensor I2C HTU**********************************
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
/************************mq2sensor************************************/
#define INTERVAL_READ_GAS         20000
#define INTERVAL_READ_HTU         10000
#define INTERVAL_STATE_UPDATE     60000
#define INTERVAL_NTP_UPDATE       6400000
#define INTERVAL_IOTDATA_UPDATE   30000
#define INTERVAL_NORMAL_RESET     3600000


unsigned long time_Read_Gas;
unsigned long time_IoTDATA_Update;
unsigned long time_Read_HTU;

/************************Hardware Related Macros************************************/
#define         MQ2PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE_MQ2                 (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR_MQ2      (9.577) //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_HYDROGEN                  (0)
#define         GAS_LPG                       (1)
#define         GAS_METHANE                   (2)
#define         GAS_CARBON_MONOXIDE           (3)
#define         GAS_ALCOHOL                   (4)
#define         GAS_SMOKE                     (5)
#define         GAS_PROPANE                   (6)
#define         accuracy                      (0)   //for linearcurves
//#define         accuracy                    (1)   //for nonlinearcurves, un comment this line and comment the above line if calculations 
                                                    //are to be done using non linear curve equations
/*****************************Globals************************************************/
float           Ro = 0;                            //Ro is initialized to 10 kilo ohms

volatile int READ_GAS_HYDROGEN =        0;
volatile int READ_GAS_LPG =             0;
volatile int READ_GAS_METHANE =         0;
volatile int READ_GAS_CARBON_MONOXIDE = 0;
volatile int READ_GAS_ALCOHOL =         0;
volatile int READ_GAS_SMOKE =           0;
volatile int READ_GAS_PROPANE =         0;

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE_MQ2*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                     //take multiple samples
    RS_AIR_val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;              //calculate the average value

  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ2;                      //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                 //according to the chart in the datasheet 

  return r0; 
}
//************************************************FSM Settings****************************************************
#define STATE_IDLE                    0
#define STATE_READ_TEMPERATURE_DATA   1
#define STATE_READ_HUMIDITY_DATA      2
#define STATE_UPDATE_UI_SCREEN_HTU    3
#define STATE_READ_GASES_DATA         4
#define STATE_UPDATE_UI_SCREEN_MQ2    5
#define STATE_TRANSMIT_DATA           6
#define STATE_UPDATE                  7
#define STATE_TRANSMIT_ALARM_UPDATE   8
#define STATE_TRANSMIT_DEVICE_UPDATE  9
#define STATE_UPDATE_TIME             10
int fsm_state;


//----------------------------------------------------------------------Funcion encargada de subscribir el nodo a los servicio de administracion remota y de notificar los para metros configurables al mismo
void initManagedDevice() {
  if (client.subscribe("iotdm-1/response")) {                         //Subscribir el nodo al servicio de mensajeria de respuesta
    Serial.println(F("subscribe to responses OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to responses FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
  }
  
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  
  if (client.subscribe("iotdm-1/device/update")) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to update OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to update FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
  }
  
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["UInterval"] = UInterval;
  metadata["UPDATETIME"] = 60*UInterval;
  metadata["NResetTIME"] = 60*60*UInterval;
  metadata["timeZone"] = timeZone;    
  JsonObject& supports = d.createNestedObject("supports");
  supports["deviceActions"] = true;  
  JsonObject& deviceInfo = d.createNestedObject("deviceInfo");
  deviceInfo["ntpServerName"] = ntpServerName;
  deviceInfo["server"] = MQTTServer;
  deviceInfo["MacAddress"] = Smacaddrs;
  deviceInfo["IPAddress"]= Sipaddrs;    
  char buff[500];
  root.printTo(buff, sizeof(buff));
  Serial.println(F("publishing device manageTopic metadata:"));
  Serial.println(buff);
  sent++;
  if (client.publish(manageTopic, buff)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
  }
}

//----------------------------------------------------------------------send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//----------------------------------------------------------------------Funcion para obtener el paquee de TP y procesasr la fecha hora desde el servidor de NTP
time_t getNtpTime(){
  while (udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      NTP = true;
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}


//----------------------------------------------------------------------anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("empezando"));
  Purpura.COn();
  if (!  wifiManager.autoConnect("flatwifIoT")) {
    Purpura.CFlash();
    if (!wifiManager.startConfigPortal("flatwifIoT")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5 * UInterval);
    }
  }
}

//*************************************************Funcion de Inicializacion de Rutina**************************
void setup() {
  pinMode(beep, OUTPUT);                                                              //Pin mapeado a pueto D5 para hacer sonar la bocina se declara como de salida
  digitalWrite(beep, LOW);                                                            //se inicializa el puerto D5 en estado "LOW" 
  Blanco.COff();                                                                      //Se incializa los puertos del RGB como "LOW"
  //-----------------------------------------------iniciar el bus I2C para los sensores y pantalla---------------
  Wire.begin(D2 , D1);                                                                //Se mapean los puertos SDA , SCL a los pines D1 y D2
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);                                          // se incia el OLED I2C en el addr 0x3C (for the 128x32)
  display.clearDisplay();                                                             // se limpia el buffer de  la pantalla OLED
  delay(2000);
  //-----------------------------------------------Configuracion de posicion y tipo de letra y color incial de la pantalla OLED para DEbug-----
  display.setTextSize(1);                                                             //
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Screen READY!");
  display.display();

  Serial.begin(115200);
  Serial.println(F("")); 
  Serial.println(F("Inicializacion de programa de boton con identificacion RFID;"));
  Serial.println(F("Parametros de ambiente de funcionamiento:"));
  Serial.print(F("            CHIPID: "));
  Serial.println(NodeID);
  Serial.print(F("            HARDWARE: "));
  Serial.println(HardwareVersion);
  Serial.print(F("            FIRMWARE: "));
  Serial.println(FirmwareVersion);
  Serial.print(F("            Servidor de NTP: "));
  Serial.println(ntpServerName);
  Serial.print(F("            Servidor de MQTT: "));
  Serial.println(MQTTServer);
  Serial.print(F("            Client ID: "));
  Serial.println(clientId); 
  delay(UInterval);

  Serial.println("HTU21D-F test");
  

  if (!htu.begin()) {
    Serial.println("Couldn't find HTU sensor!");
    display.println("Couldn't find HTU sensor!");
    display.display();
    while (1);
  }
  
  Serial.println("htu Ready");
  display.println("htu Ready");
  display.display();
  
  Serial.println("Calibrating...\n");
  display.println("Calibrating...");
  display.display();
  Ro = MQCalibration(MQ2PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
  //when you perform the calibration
  Serial.println("Calibration is done...\n");
  display.println("Calibration is done..."); 
  display.display();
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  //--------------------------------------------------------------------------Configuracion Automatica de Wifi   
  while (WiFi.status() != WL_CONNECTED) {                                   //conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.clearDisplay();
    display.println("AP: flatwifIOT"); 
    display.display(); 
    wifimanager();
    delay(UInterval);
  }
  Serial.print(F("Wifi conectado, Direccion de IP Asignado: "));
  Serial.println(WiFi.localIP());
  Sipaddrs = WiFi.localIP().toString();
  Serial.print(F("Direccion de MAC Asignado: "));
  Serial.println(WiFi.macAddress());
  Smacaddrs = String(WiFi.macAddress());
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.clearDisplay();
  display.println(Sipaddrs);
  display.println(Smacaddrs); 
  display.display(); 
  Serial.println(F(""));                                                         //dejamos una linea en blanco en la terminal 
  //una vez contados al Wifi nos aseguramos tener la hora correcta simepre
  Serial.println(F("Connected to WiFi, sincronizando con el NTP;"));                    //mensaje de depuracion para saber que se intentara obtner la hora
  //--------------------------------------------------------------------------Configuracion de NTP
  Serial.print(F("servidor de NTP:"));
  Serial.println(ntpServerName);
  //--------------------------------------------------------------------------Configuracion de UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  while (NTP == false) {
    setSyncProvider(getNtpTime);                                                          //iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
    delay(UInterval);
  }
  NTP = false;
  //--------------------------------------------------------------------------Connectando a servicio de MQTT
  Serial.println(F("Time Sync, Connecting to mqtt sevrer"));
  display.println("Time Sync!");
  display.display(); 
  mqttConnect();                                                            //Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  Serial.println(F("Mqtt Connection Done!, sending Device Data"));
  display.println("Mqtt Connection Done!");
  display.display();
  //--------------------------------------------------------------------------Enviando datos de primera conexion
  initManagedDevice();                                                      //inciamos la administracion remota desde Bluemix
  Serial.println(F("Finalizing Setup"));                                    //enviamos un mensaje de depuracion
  Blanco.COff();
  
  display.println(" NODE READY!");
  display.display();

  // Clear the buffer.
  display.clearDisplay();
   fsm_state = STATE_IDLE; //inciar el estado del la maquina de stado finito 
}

void updateDeviceInfo(){
  msg = ("on");
  WifiSignal = WiFi.RSSI();
  if (WiFi.RSSI() < -75){
    msg = ("LOWiFi");
    Rojo.Flash();
    buzzer();
    Serial.print(WiFi.SSID());
    Serial.print(" ");
    Serial.println(WiFi.RSSI());
    fsm_state = STATE_TRANSMIT_ALARM_UPDATE; //publishRF_ID_Manejo(NodeID, msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
    return;
  }
 }

 //-------- Data de Manejo RF_ID_Manejo. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishRF_ID_Manejo (String IDModulo,String MSG,int RSSIV, int env, int fail,String Tstamp, String SMacAd, String SIpAd){
  StaticJsonBuffer<300> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& Ddata = d.createNestedObject("Ddata");
  Ddata["ChipID"] = IDModulo;
  Ddata["Msg"] = MSG;
  Ddata["RSSI"] = RSSIV;
  Ddata["publicados"] = env;
  Ddata["enviados"] = sent;
  Ddata["fallidos"] = fail;
  Ddata["Tstamp"] = Tstamp;
  Ddata["Mac"] = SMacAd;
  Ddata["Ip"] = SIpAd;
  char MqttDevicedata[300];
  root.printTo(MqttDevicedata, sizeof(MqttDevicedata));
  Serial.println(F("publishing device data to manageTopic:"));
  Serial.println(MqttDevicedata);
  sent++;
  if (client.publish(manageTopic, MqttDevicedata)) {
     Serial.println(F("enviado data de dispositivo:OK"));
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("enviado data de dispositivo:FAILED"));
    failed ++;
  }
}


//------------------------------------------------------------------------------------------------Funcion de reseteo normal
void NormalReset(){
  Serial.println(F("Ejecutando F_NormalReset "));     
  hora++;
  WifiSignal = WiFi.RSSI();
  if (hora > 24)
  {
    msg = ("24h NReset");  
    publishRF_ID_Manejo(NodeID, msg, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
    void disconnect ();
    hora = 0;
    ESP.restart();
  }
}

//----------------------------------------------------------------------------funcion que procesa como desplegar y transmitir la hora de acuerdo al formato del ISO8601
void CheckTime(){ //digital clock display of the time
  time_t prevDisplay = 0; 
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) {                                             //update the display only if time has changed
      prevDisplay = now();
      ISO8601 = String (year(), DEC);
      ISO8601 += "-";
      ISO8601 += month();
      ISO8601 += "-";
      ISO8601 += day();
      ISO8601 +="T";
      if ((hour() >= 0)&& (hour() < 10)){
        //Serial.print(F("+0:"));
        //Serial.println(hour());
        ISO8601 +="0";
        ISO8601 += hour();
      }else{
        //Serial.print(F("hora:"));
        //Serial.println(hour());
        ISO8601 += hour();
      }
      ISO8601 += ":";
      ISO8601 += minute();
      ISO8601 += ":";
      ISO8601 += second();
    }
  }
}



/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) 
  {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

//***************************************************Funcion de Despliegue de mensaje en OLED*******************
void display_Msg_Screen(String display_msg, int posx, int posy ){
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(posx, posy);
  display.println(display_msg);
  display.display();
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function uses different equations representing curves of each gas to 
         calculate the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{ 
  if ( accuracy == 0 ) {
  if ( gas_id == GAS_HYDROGEN ) {
    return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)));
  } else if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)));
  } else if ( gas_id == GAS_ALCOHOL ) {
    return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)));
  } else if ( gas_id == GAS_SMOKE ) {
    return (pow(10,((-2.331*(log10(rs_ro_ratio))) + 3.596)));
  } else if ( gas_id == GAS_PROPANE ) {
    return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)));
  }    
} 

  else if ( accuracy == 1 ) {
    if ( gas_id == GAS_HYDROGEN ) {
    return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)));
  } else if ( gas_id == GAS_LPG ) {
    return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)));
  } else if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)));
  } else if ( gas_id == GAS_ALCOHOL ) {
    return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)));
  } else if ( gas_id == GAS_SMOKE ) {
    return (pow(10,(-0.976*pow((log10(rs_ro_ratio)), 2) - 2.018*(log10(rs_ro_ratio)) + 3.617)));
  } else if ( gas_id == GAS_PROPANE ) {
    return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)));
  }
}    
  return 0;
}

void Get_PPM_Compsosition(){
  
  READ_GAS_HYDROGEN = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_HYDROGEN);
  //Serial.print ("HYDROGEN:" + String(READ_GAS_HYDROGEN)  + "ppm    ");
  display_Msg_Screen("H: " + String(READ_GAS_HYDROGEN) + "ppm" , 0, 0);
  
  READ_GAS_LPG = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_LPG);
  //Serial.print ("LPG:" + String(READ_GAS_LPG)  + "ppm    ");
  display_Msg_Screen("LPG: " + String(READ_GAS_LPG) + "ppm" , 45, 0);
  
  READ_GAS_METHANE = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_METHANE);
  //Serial.print ("CH4:" + String(GAS_METHANE)  + "ppm    ");
  display_Msg_Screen("CH4: " + String(GAS_METHANE) + "ppm" , 0, 8);

  READ_GAS_CARBON_MONOXIDE = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_CARBON_MONOXIDE);
  //Serial.print ("CARBON_MONOXIDE: " + String(READ_GAS_CARBON_MONOXIDE)  + "ppm    ");
  display_Msg_Screen("CO: " + String(READ_GAS_CARBON_MONOXIDE) + "ppm" , 45, 8);
  
  READ_GAS_ALCOHOL = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_ALCOHOL);
  //Serial.print ("GAS_ALCOHOL: " + String(READ_GAS_ALCOHOL)  + "ppm    ");
  display_Msg_Screen("CH3OH: " + String(READ_GAS_ALCOHOL) + "ppm" , 0, 16);

  READ_GAS_SMOKE = MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_SMOKE);
  //Serial.print ("SMOKE: " + String(READ_GAS_SMOKE)  + "ppm    ");
  display_Msg_Screen("SMOKE: " + String(READ_GAS_SMOKE) + "ppm" , 45, 16);
    
  READ_GAS_PROPANE= MQGetGasPercentage(MQRead(MQ2PIN)/Ro,GAS_PROPANE);
  //Serial.print ("PROPANE:" + String(READ_GAS_PROPANE)  + "ppm    ");
  display_Msg_Screen("C3H8: " + String(READ_GAS_PROPANE) + "ppm" , 0, 25);
  display.clearDisplay();  
}
//***************************************************Funcion de Promedio de lectura de Sensor*******************
float get_SensorReading(float Sensor_Reading){
  uint8_t i;
  float averageReading = 0;
  float sampleReading = 0;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   sampleReading = sampleReading + Sensor_Reading;
   delay(100);
  }
  
  // average all the samples out
  averageReading = sampleReading/NUMSAMPLES;
  //debug massage: 
  //Serial.print("Average Sensor reading : "); 
  //Serial.println(averageReading);
  display.clearDisplay(); 
  return averageReading;
}

//****************************************************funcion de enviode Datos***********************************
void publishRF_Boton(String IDModulo, float TEMPERATURA, float HUMEDAD, int HIDROGENO, int LPG, int METANO, int MONOXIDO_DE_CARBONO, int ALCOHOL, int HUMO, int PROPANO , String Tstamp) {
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& iotdata = d.createNestedObject("iotdata");
  iotdata["ChipID"] = IDModulo;
  iotdata["TEMPERATURA"] = TEMPERATURA;
  iotdata["HUMEDAD"] = HUMEDAD;
  iotdata["HIDROGENO"] = HIDROGENO;
  iotdata["LPG"] = LPG;
  iotdata["METANO"] = METANO;
  iotdata["MONOXIDO_DE_CARBONO"] = MONOXIDO_DE_CARBONO;
  iotdata["ALCOHOL"] = ALCOHOL;
  iotdata["HUMO"] = HUMO;
  iotdata["PROPANO"] = PROPANO;
  iotdata["Tstamp"] = Tstamp;
  char MqttIoTdata[500];
  root.printTo(MqttIoTdata, sizeof(MqttIoTdata));
  Serial.println(F("publishing device publishTopic metadata:")); 
  Serial.println(MqttIoTdata);
  sent ++;
  if (client.publish(publishTopic, MqttIoTdata)){
    Serial.println(F("enviado data de boton: OK"));
    Verde.Flash();
    buzzer();
    published ++;
    failed = 0; 
  }else {
    Serial.println(F("enviado data de boton: FAILED"));
    Rojo.Flash();
    failed ++;
  }
  Blanco.COff();
}

 
//**************************************************Funcion de Rutina*******************************************
void loop() {
  switch(fsm_state)
  { // inciar el casw switch
    case STATE_IDLE: // hacer cuando el estado sea IDLE
    
     if(millis() > time_Read_HTU + INTERVAL_READ_HTU)
    {
      time_Read_Gas = millis();
      fsm_state = STATE_READ_TEMPERATURE_DATA;  
    }
    
    if(millis() > time_Read_Gas + INTERVAL_READ_GAS)
    {
      time_Read_Gas = millis();
      display.clearDisplay();
      Get_PPM_Compsosition();    
    }

    if(millis() > time_Normal_Reset + INTERVAL_NORMAL_RESET)
    {
      time_Read_Gas = millis();
      NormalReset();    
    }

    if(millis() > last_State_Update + INTERVAL_STATE_UPDATE)
    {
      last_State_Update = millis(); //Actulizar la ultima hora de envio
      fsm_state = STATE_UPDATE;
    }
    
    if(millis() > last_NTP_Update + INTERVAL_NTP_UPDATE) 
    {
      last_NTP_Update = millis(); //Actulizar la ultima hora de envio
      fsm_state = STATE_UPDATE_TIME;
    }

     if(millis() > time_IoTDATA_Update + INTERVAL_IOTDATA_UPDATE) 
    {
      time_IoTDATA_Update = millis(); //Actulizar la ultima hora de envio
      fsm_state = STATE_TRANSMIT_DATA;
    }

    // VERIFICAMOS CUANTAS VECES NO SE HAN ENVIOADO PAQUETES (ERRORES)
    if (failed >= FAILTRESHOLD)
    {
      failed =0;
      published =0;
      sent=0;    
      ESP.restart();
    }
    
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.connected()) 
    {
      MQTTreconnect();
    }
    client.loop();
    break;

    case STATE_READ_TEMPERATURE_DATA:                                             //Se verifica la lectura de la temperatura en el htu
    HTU_Temp = get_SensorReading(htu.readTemperature());
    
    if(Old_HTU_Temp != HTU_Temp)
    {
      Old_HTU_Temp = HTU_Temp;
      New_screen_display = true;
    }
    fsm_state = STATE_READ_HUMIDITY_DATA;
    break;

    case STATE_READ_HUMIDITY_DATA:                                                 //Se verifica la lectura del la humedad en el htu
    HTU_Hum =  get_SensorReading(htu.readHumidity());

    if(Old_HTU_Hum != HTU_Hum)
    {
      Old_HTU_Hum = HTU_Temp;
      New_screen_display = true;  
    }

    fsm_state = STATE_UPDATE_UI_SCREEN_HTU;
    break;

    case STATE_UPDATE_UI_SCREEN_HTU:                                                //Se Actuliza la pantalla del Oled si es que hubo un cambio en las lecturas
    
    if(New_screen_display == true)
    {
      New_screen_display = false;
      display.clearDisplay();
      display_Msg_Screen("Humedad: " + String (HTU_Hum) + "%" , 0, 8);
      display_Msg_Screen("Temperatura: " + String (HTU_Temp) + "*C" , 0, 0);
    }

    fsm_state = STATE_IDLE;
    break;
    
    case STATE_TRANSMIT_DATA: //Si se presiono el boton
    //Check connection
    //Send the data
    Serial.println(F("STATE_TRANSMIT_DATA"));
    CheckTime();
    publishRF_Boton(NodeID, HTU_Temp, HTU_Hum, READ_GAS_HYDROGEN, READ_GAS_LPG, READ_GAS_METHANE, READ_GAS_CARBON_MONOXIDE, READ_GAS_ALCOHOL, READ_GAS_SMOKE, READ_GAS_PROPANE  ,ISO8601);  // publishRF_Boton(String IDModulo, String EventID, String Tstamp)
    fsm_state = STATE_IDLE;
    break; 
    
    case STATE_UPDATE:
    Serial.println(F("STATE_UPDATE"));
    updateDeviceInfo();
    fsm_state = STATE_TRANSMIT_DEVICE_UPDATE;
    break;
    
    case STATE_TRANSMIT_DEVICE_UPDATE:
    Serial.println(F("STATE_TRANSMIT_DEVICE_UPDATE"));
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.connected())
    {
      MQTTreconnect();
    }
    //verificar la hora
    CheckTime();
    publishRF_ID_Manejo(NodeID, msg, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);
    fsm_state = STATE_IDLE;
    break;
    
    case STATE_TRANSMIT_ALARM_UPDATE:
     Serial.println(F("STATE_TRANSMIT_ALARM_UPDATE"));
     //verificar que el cliente de Conexion al servicio se encuentre conectado
     if (!client.connected())
     {
       MQTTreconnect();
     }
     // Verificar la hora
     CheckTime();
     publishRF_ID_Manejo(NodeID,msg, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);
    break;
    
    case STATE_UPDATE_TIME:
     Serial.println(F("STATE_UPDATE_TIME"));
     Serial.println(F("Starting UDP"));
     udp.begin(localPort);
     Serial.print(("Local port: "));
     Serial.println(udp.localPort());
     while (NTP == false) 
     {
       setSyncProvider(getNtpTime);                                                          //iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
       delay(UInterval);
     }                                                   //Cuando fue actualizada la hora del reloj
     NTP = false;
     fsm_state = STATE_IDLE; 
    break;
  }
 
  yield();
}
