/*Copyright <2018> <Edwin Kestler>
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/

#include <WString.h>                // include the String library
#include <IPAddress.h>              // include the IPAdress library
//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "FLATBOX"
#define DEVICE_TYPE "WEMOSD1MINIV2"
#define DEVICE_ID "TEST_BLANK"
#define TOKEN "NONE_TK"
//-------- Customise the above values --------

#define InternetServer   "eospower.flatbox.io"
//#define adnode      "adnode.flatbox.io"

//-------- Customise these values-----------

char MQTTServer [] = InternetServer;
String FirmwareVersion= "V1.0";                                        //read in chage history
String HardwareVersion= "V1.0";                                        //read in chage history 
//---------Blurmix Topics---------------------

const char  publishTopic[] =  "iot-2/evt/status/fmt/IoTdata/json";
const char  responseTopic[] = "iotdm-1/IoTdata/response/";
const char  manageTopic[] =   "iotdevice-1/mgmt/IoTdata/manage";
const char  updateTopic[] =   "iotdm-1/device/IoTdata/update";
const char  rebootTopic[] =   "iotdm-1/mgmt/initiate/device/IoTdata/reboot";

//-----------Variables de Configuracion del Servicio de NTP
//-------- Configuracion de parametros de servicio remots de hora (NTP Servers:)
#if defined (InternetServer)
       IPAddress  timeServer(129,  6, 15, 29); // time.nist.gov NTP ;
       const char* ntpServerName = "129.6.15.29"; //const char*;
       unsigned int localPort = 2390;  // local port to listen for UDP packets
       const int timeZone = -6;  // Eastern central Time (USA)
#else
      IPAddress timeServer(129,  6, 15, 29); // time.nist.gov NTP server IPAddress timeServer(192,168,120,211);
      const char* ntpServerName = "129.6.15.29"; //const char* ntpServerName = "192.168.120.211";
      unsigned int localPort = 2390;  // local port to listen for UDP packets
      const int timeZone = -6;  // Eastern central Time (USA)
#endif


//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long UInterval     = 1000UL; //Variable configurable remotamente sobre el interbalo de publicacion

//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150