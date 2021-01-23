/***
 COM Port 7 
   ESPNOW - Honda Remote Start using ESPNOW - Slave
   Date: 9th July 2020
   Author: Stein Espe
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32 to control a Hona EU70IS
   Description: This sketch consists of the code for the Master module.
   Communication between Master and Slave use ESPNow-protocol, 
   fixed MAC eddresses entrered in code
   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Set fixed MAC and add slave
   Step 3 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Set fixed MAC and add slave
   Step 3 : Register for receive callback and wait for data
   Step 5 : Once data arrives, decode and set relay's

   Code based on example from this source:
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> //NEED THIS TO COMPILE
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include "uptime_formatter.h"

#define WIFI_CHANNEL 1
#define SWVERSION "ESPNow/Honda Master Remote control VictronVenus with OAT V1.01" 
// Onboard ledpin

// Debug flag
boolean DEBUG =false;
boolean NETDEBUG =false;


esp_now_peer_info_t slave;
const esp_now_peer_info_t *slaveNode = &slave;
const byte maxDataFrameSize = 200;
uint8_t dataToSend[maxDataFrameSize];
byte cnt=0;
esp_err_t sendResult;
/* -----------------Declarations-----------------*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

//  Input pin
#define pinHondaStartedFB 13                // pin for Honda StartedFB (LED)
#define pinHondaStartedFBOnboard 2          // pin for Honda StartedFB (LED)
#define pinHondaStart 15                    // pin for Honda Start/Crank
#define pinHondaManualStart 16              // pin for Honda Manual Start
bool gblSlaveHondaStart = false ;          // default global, HondaStart=false = off.  Will contain Slave's reported status
bool gblSlaveHondaStarting = false ;          // default global, HondaStarting=false = off.  Will contain Slave's reported status during starting
bool gbllocHondaStart = false;
// Statistics
int   gblSlaveHondaStartCount=0;                  // Recorded start count
int   gblESPUptime;                         // Recorded uptime in hours
#define HondaRestartTime 30000   // ms Restart blocking time
unsigned long startTime=0;

// Debounce var's
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros=micros();

// Structure example to receive data
// Must match the receiver structure
typedef struct struct_mastermessage {
  char a[32];
  bool HondaRunningFB;
  bool HondaIgnitionOn;
  bool HondaStart;
} struct_mastermessage;
// Create a struct_message called myData for master message to slave
struct_mastermessage myData;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_slavemessage {
  char a[32];
  bool HondaIgnitionOn;
  bool HondaStarting;
  bool HondaRunning;
  int  HondaStartCount;
} struct_slavemessage;
// Create a struct_message called myData for master message to slave
struct_slavemessage myDataSlave;


//  Set legal MAC address; observe success on setting MAC in serioal monitor
//
const uint8_t slaveCustomMac[] = {0x30, 0xAE, 0xA4, 0x07, 0xEA, 0xA9}; //Custom mac address for The Mester Device 
const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x0B, 0x9E, 0x7D}; //Custom mac address for The Slave Device 

// Network credentials ESP32 AP HomdaMaster
const char* ssid     = "HondaMaster";
const char* password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

//
//  Set fixed MAC address and set channel to value WIFI_CHANNEL
//  Checks status with ESP_ERROR_CHECK 
//

void initVariant()
{
  uint8_t primaryChan;
  wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
//  esp_log_level_set("wifi", ESP_LOG_VERBOSE);      // enable WARN logs from WiFi stack
 
  Serial.println("InitVariant"); // esp_base_mac_addr_set
  WiFi.mode(WIFI_MODE_STA); // WIFI_STA Mode
//  ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, &masterCustomMac[0]));   // #include <esp_wifi.h> required to compile this line
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  Serial.print("Get_channel Status: "); Serial.println(primaryChan);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, secondChan));
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
  Serial.print("Set_channel Status: "); Serial.println(primaryChan);
 //     const uint8_t protocol = WIFI_PROTOCOL_LR;
 //     ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_AP, protocol) );
}

void addSlaveAsPeer()
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           slaveCustomMac[0], slaveCustomMac[1], slaveCustomMac[2], slaveCustomMac[3], slaveCustomMac[4], slaveCustomMac[5]);
  memcpy( &slave.peer_addr, &slaveCustomMac, 6 );
  slave.channel = WIFI_CHANNEL;
  slave.encrypt = 0;
  slave.ifidx= ESP_IF_WIFI_AP;
  const uint8_t *peer_addr = slave.peer_addr;
  if( esp_now_del_peer(peer_addr) == ESP_OK)
  {
    Serial.print("Slave deleted Peer!, MAC:");
    Serial.println(macStr);
  }  
  if( esp_now_add_peer(slaveNode) == ESP_OK)
  {
    Serial.print("Slave Added As Peer!, MAC:");
    Serial.println(macStr);
  }
}
//
// Init ESP Now with fallback
//
void InitESPNow() {
  Serial.print("Mac this station:");Serial.println(WiFi.macAddress() );
    WiFi.mode(WIFI_AP);
    //If the initialization was successful
    if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    }
    //If there was an error
    else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
    }
}
//
// send data
void sendData() {
    // Set values to send
  strcpy(myData.a, "Master-->Slave");
  myData.HondaRunningFB = false;
  myData.HondaIgnitionOn = digitalRead(pinHondaManualStart);
  myData.HondaStart = gbllocHondaStart;
//  gbllocHondaStart =  !digitalRead(pinHondaStart);
  if (DEBUG){
  Serial.println("Routine sendData");
  Serial.print("  HondaStart: ");
  Serial.println(myData.HondaStart);
  }
    if (NETDEBUG){
  TelnetStream.println("Routine sendData");
  TelnetStream.print("  HondaStart: ");
  TelnetStream.println(myData.HondaStart);
  }
  const uint8_t *peer_addr = slave.peer_addr;
  ESP_ERROR_CHECK(esp_now_send(peer_addr, (uint8_t *) &myData, sizeof(myData)));
}
//
// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (DEBUG) {
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

// callback when data is recv from Slave
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (DEBUG) {
  Serial.println("Routine OnDataRecv..");
  Serial.print("  Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("  Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
  }
  if (NETDEBUG) {
  TelnetStream.println("Routine OnDataRecv..");
  TelnetStream.print("  Last Packet Recv from: "); TelnetStream.println(macStr);
  TelnetStream.print("  Last Packet Recv Data: "); TelnetStream.println(*data);
  TelnetStream.println("");
  }
  int indata= *data;
  memcpy(&myDataSlave, data, sizeof(myDataSlave));
  gblSlaveHondaStart=myDataSlave.HondaRunning;
  gblSlaveHondaStarting=myDataSlave.HondaStarting;
  gblSlaveHondaStartCount=myDataSlave.HondaStartCount;
  if(DEBUG) {
     Serial.print("  gbllocHondaStartReceived:");
     Serial.println(gbllocHondaStart);
     Serial.print("  gblSlaveHondaStartReceived:");
     Serial.println(gblSlaveHondaStart);
     Serial.print("  Bytes received: ");
     Serial.println(data_len);
     Serial.print("  Char: ");
     Serial.println(myDataSlave.a);
     Serial.print("  HondaRunning: ");
     Serial.println(myDataSlave.HondaRunning);
     Serial.print("  HondaIgnitionOn: ");
     Serial.println(myDataSlave.HondaIgnitionOn);
     Serial.print("  HondaStarting: ");
     Serial.println(myDataSlave.HondaStarting);
     Serial.println();
    }
  if(NETDEBUG) {
     TelnetStream.print("  gbllocHondaStartReceived:");
     TelnetStream.println(gbllocHondaStart);
     TelnetStream.print("  gblSlaveHondaStartReceived:");
     TelnetStream.println(gblSlaveHondaStart);
     TelnetStream.print("  Bytes received: ");
     TelnetStream.println(data_len);
     TelnetStream.print("  Char: ");
     TelnetStream.println(myDataSlave.a);
     TelnetStream.print("  HondaRunning: ");
     TelnetStream.println(myDataSlave.HondaRunning);
     TelnetStream.print("  HondaIgnitionOn: ");
     TelnetStream.println(myDataSlave.HondaIgnitionOn);
     TelnetStream.print("  HondaStarting: ");
     TelnetStream.println(myDataSlave.HondaStarting);
     TelnetStream.println();
    }
  if(gblSlaveHondaStart) {
     digitalWrite(pinHondaStartedFB,HIGH);   
     digitalWrite(pinHondaStartedFBOnboard,HIGH);   
  }
  if(!gblSlaveHondaStart) {
     digitalWrite(pinHondaStartedFB,LOW);   
     digitalWrite(pinHondaStartedFBOnboard,LOW);   
  }
}

/* interrupt function toggle the Start automatic */
void IRAM_ATTR intr_HondaStart() {
gbllocHondaStart = (digitalRead(pinHondaManualStart) or !digitalRead(pinHondaStart));
}

/* interrupt function toggle the Start automatic */
void IRAM_ATTR intr_HondaManualStart() {
// gbllocHondaStart = (digitalRead(pinHondaManualStart) or !digitalRead(pinHondaStart));
 gbllocHondaStart = ( !digitalRead(pinHondaStart));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(SWVERSION);
  // print info
  Serial.print("Honda start input pin on :");
  Serial.println(pinHondaStart);
  Serial.print("Honda ignition on pin on :");
  Serial.println(pinHondaManualStart);  
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  //
  // Connect to Wi-Fi network with SSID and password
  //
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password,WIFI_CHANNEL);
  Serial.println("Wait 100 ms for AP_START...");
  delay(100);
  // Set AP parameters
  Serial.println("Set softAPConfig");
  IPAddress Ip(192, 168, 10, 71);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
  IPAddress IP = WiFi.softAPIP();
   // This is the HW-mac addres, mac address of the node in AP Mode and IP address
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("Channel: "); Serial.println(WiFi.channel()); 
  Serial.print("AP IP address: ");Serial.println(IP);
  Serial.print("MAC address: ");  Serial.println(WiFi.macAddress());
  
  server.begin();
// Telnet stream start
  TelnetStream.begin();
  
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setHostname("HondaMaster");

  // No authentication by default
  // ArduinoOTA.setPassword("");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      esp_now_deinit();
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      TelnetStream.print("Progress: ");TelnetStream.print(progress / (total / 100));TelnetStream.print("\r");
      })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  //
  // ESP Now setup
  //
  initVariant();
  InitESPNow();
  addSlaveAsPeer();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  //
  // Set pin mode & attach interrupt
  //
  pinMode(pinHondaStart,INPUT);  // Honda  Start / Stopp fra Victron Venus rele 0
  attachInterrupt(pinHondaStart, intr_HondaStart, CHANGE);
  gbllocHondaStart = !digitalRead(pinHondaStart);
  pinMode(pinHondaManualStart,INPUT_PULLUP);  // Honda ingnition ON 
  attachInterrupt(pinHondaManualStart, intr_HondaManualStart, CHANGE);
  pinMode(pinHondaStartedFB,OUTPUT);  // Honda startedFB 
  pinMode(pinHondaStartedFBOnboard,OUTPUT);  // Honda startedFB   
  digitalWrite(pinHondaStartedFB,LOW);
  digitalWrite(pinHondaStartedFBOnboard,LOW);
// Wifi status
    WiFi.printDiag(Serial);
}

void loop()
{
    ArduinoOTA.handle();
    WiFiClient client = server.available();   // Listen for incoming clients

// Check for change in input;  done in loop to avoid ripple, and evaluate legality of start/stop
// Rules
//  Stop  :  Always legal
//  Start :   Legal when not  gbllocHondaStart ( Master's knowledge of status) 
//            and
//            Not blocked by rapis start requests (time in HondaRestartTime) 

//  if((gbllocHondaStart !=gblSlaveHondaStart) &&  (millis() - startTime >=HondaRestartTime)){
  if( millis() - startTime >=HondaRestartTime){
//    gbllocHondaStart=gblSlaveHondaStart;
      startTime=millis();
      if(DEBUG) {
        Serial.print(" gbllocHondaStart :");Serial.println(gbllocHondaStart);
        Serial.print(" gblSlaveHondaStart :");Serial.println(gblSlaveHondaStart);
        Serial.print(" Pin pinHondaStart :");Serial.println(!digitalRead(pinHondaStart));
        Serial.print(" Pin pinHondaStartedFB :");Serial.println(digitalRead(pinHondaStartedFB));
        Serial.print(" Pin pinHondaManualStart :");Serial.println(digitalRead(pinHondaManualStart));
      }
      sendData();
  }
if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  switch (Serial.read()) {
    case 'M':
        Serial.println("Serial menu .. :");
        Serial.println(" M - menu");
        Serial.println(" S - Status");
        Serial.println(" A - Address");
        Serial.println(" T - Toggle debug");
        Serial.println(" R - Reboot");
        break;
    case 'S':
        Serial.println("Status .. :");
        Serial.print(" ");Serial.println(SWVERSION);
        Serial.print(" gbllocHondaStart        :");Serial.println(gbllocHondaStart);
        Serial.print(" gblSlaveHondaStart      :");Serial.println(gblSlaveHondaStart);
        Serial.print(" gblSlaveHondaStarting   :");Serial.println(gblSlaveHondaStarting);
        Serial.print(" gblSlaveHondaStartCount :");Serial.println(gblSlaveHondaStartCount);     
        Serial.print(" Pin pinHondaStart       :");Serial.println(!digitalRead(pinHondaStart));
        Serial.print(" Pin pinHondaStartedFB   :");Serial.println(digitalRead(pinHondaStartedFB));
        Serial.print(" Pin pinHondaManualStart :");Serial.println(digitalRead(pinHondaManualStart));     
        Serial.println(" Uptime : " + uptime_formatter::getUptime());
        Serial.print(" Free mem                :");Serial.println(ESP.getFreeHeap());
        Serial.print(" Debuglevel              :");Serial.println(DEBUG);
        break;
    case 'T':
        DEBUG = !DEBUG;
        break;
    case 'R':
        ESP.restart();
        break;
    case 'A':
      // This is the HW-mac addres, mac address of the node in AP Mode and IP address
        Serial.println("Address .. :");
        IPAddress IP = WiFi.softAPIP();
      Serial.print(" AP IP address: ");Serial.println(IP);
      Serial.print(" MAC address: ");  Serial.println(WiFi.macAddress());
      Serial.print(" Channel: "); Serial.println(WiFi.channel()); 
      Serial.print(" AP MAC: "); Serial.println(WiFi.softAPmacAddress());
      break;
  }
  switch (TelnetStream.read()) {
    case 'M':
        TelnetStream.println("Serial menu .. :");
        TelnetStream.println(" M - menu");
        TelnetStream.println(" S - Status");
        TelnetStream.println(" A - Address");
        TelnetStream.println(" T - Toggle debug");
        TelnetStream.println(" R - Reboot");
        break;
    case 'S':
        TelnetStream.println("Status .. :");
        TelnetStream.println(SWVERSION);
        TelnetStream.print(" gbllocHondaStart          :");TelnetStream.println(gbllocHondaStart);
        TelnetStream.print(" gblSlaveHondaStart        :");TelnetStream.println(gblSlaveHondaStart);
        TelnetStream.print(" gblSlaveHondaStartCount   :");TelnetStream.println(gblSlaveHondaStartCount);     
        TelnetStream.print(" Pin pinHondaStart         :");TelnetStream.println(!digitalRead(pinHondaStart));
        TelnetStream.print(" Pin pinHondaStartedFB     :");TelnetStream.println(digitalRead(pinHondaStartedFB));
        TelnetStream.print(" Pin pinHondaManualStart   :");TelnetStream.println(digitalRead(pinHondaManualStart));     
        TelnetStream.println(" Uptime : " + uptime_formatter::getUptime());
        TelnetStream.print(" Free mem");TelnetStream.println(ESP.getFreeHeap());
        TelnetStream.print(" Debuglevel                :");TelnetStream.println(NETDEBUG);
        break;
    case 'T':
        NETDEBUG = !NETDEBUG;
        break;
    case 'R':
        ESP.restart();
        break;
    case 'A':
      // This is the HW-mac addres, mac address of the node in AP Mode and IP address
        TelnetStream.println("Address .. :");
        IPAddress IP = WiFi.softAPIP();
      TelnetStream.print(" AP IP address: ");TelnetStream.println(IP);
      TelnetStream.print(" MAC address: ");  TelnetStream.println(WiFi.macAddress());
      TelnetStream.print(" Channel: "); TelnetStream.println(WiFi.channel()); 
      TelnetStream.print(" AP MAC: "); TelnetStream.println(WiFi.softAPmacAddress());
      break;
  }

}
 
