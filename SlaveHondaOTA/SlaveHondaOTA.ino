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
 
// Onboard ledpin

#define LED 4 
#define pinHondaStart 13
#define pinHondaIgnitionOn 14
#define pinHondaRunningFB 15
#define HW_HondaFB  0  // HW feedback enabled

#define HondaCrankTime 3000   // ms crank time
#define HondaSendStatusTime 10000   // ms Slave send interval
unsigned long startTime=0;
#define SWVERSION "ESPNow/Honda Slave Remote control VictronVenus with OAT V1.01" 
// Onboard ledpin

// Debug flag
boolean DEBUG =false;
boolean NETDEBUG =false;

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


bool gblHondaRunningFB= false;
bool gblHondaStarting= false;
bool gblHondaStartCommand= false;
bool gblHondaIgnitionOn = false;
int   gblSlaveHondaStartCount=0;                  // Recorded start count
//  Set legal MAC address; observe success on setting MAC in serioal monitor
const uint8_t slaveCustomMac[] = {0x30, 0xAE, 0xA4, 0x07, 0xEA, 0xA9}; //Custom mac address for The Slave Device 
// firkantet masterkort const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x4C, 0xB9, 0xA0}; //Custom mac address for The Slave Device 
const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x0B, 0x9E, 0x7D}; //Custom mac address for The Slave Device 


// Prod const uint8_t slaveCustomMac[] = {0x30, 0xAE, 0xA4, 0x1A, 0xAE, 0x36};  //Custom mac address for This Slave Device
// Prodconst uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x89, 0x92, 0x71}; //Custom mac address for The Master Device 
esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
const byte maxDataFrameSize = 200;
uint8_t dataToSend[maxDataFrameSize];
byte cnt=0;
esp_err_t sendResult;
/* -----------------Declarations-----------------*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);


// Replace with your network credentials
const char* ssid     = "HondaSlave";
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
//  ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, &slaveCustomMac[0]));   // #include <esp_wifi.h> required to compile this line
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  Serial.print("Get_channel Status: "); Serial.println(primaryChan);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, secondChan));
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
  Serial.print("Set_channel Status: "); Serial.println(primaryChan);


 //     const uint8_t protocol = WIFI_PROTOCOL_LR;
 //     ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, protocol) );
}

void addMasterAsPeer()
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           masterCustomMac[0], masterCustomMac[1], masterCustomMac[2], masterCustomMac[3], masterCustomMac[4], masterCustomMac[5]);
  memcpy( &master.peer_addr, &masterCustomMac, 6 );
  master.channel = WIFI_CHANNEL;
  master.encrypt = 0;
  master.ifidx= ESP_IF_WIFI_AP;
  const uint8_t *peer_addr = master.peer_addr;
  if( esp_now_del_peer(peer_addr) == ESP_OK)
  {
    Serial.print("Master deleted Peer!, MAC:");
    Serial.println(macStr);
  }
  if( esp_now_add_peer(masterNode) == ESP_OK)
  {
    Serial.print("Master Added As Peer!, MAC:");
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



void setup() {
  Serial.begin(115200);
  Serial.println("Setup starts....");
  // Set pin mode SlaveUnit
  pinMode(LED,OUTPUT);
  //  Ignition out
  pinMode(pinHondaIgnitionOn,OUTPUT);
  digitalWrite(pinHondaIgnitionOn,HIGH);
  delay(100);
  // Start out
  pinMode(pinHondaStart,OUTPUT);
  digitalWrite(pinHondaStart,HIGH);  
  delay(100);
  // RunningFeedback
  #ifdef HW_HondaFB
    pinMode(pinHondaRunningFB,INPUT);
    gblHondaRunningFB = !digitalRead(pinHondaRunningFB);  
    delay(100);
  #endif
  //
  // Connect to Wi-Fi network with SSID and password
  //
//  WiFi.mode(WIFI_AP);
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password,WIFI_CHANNEL);
  Serial.println("Wait 100 ms for AP_START...");
  delay(1000);
  // Set AP parameters
  Serial.println("Set softAPConfig");
  IPAddress Ip(192, 168, 1, 80);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
  IPAddress IP = WiFi.softAPIP();
   // This is the HW-mac addres, mac address of the node in AP Mode and IP address
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("Channel: "); Serial.println(WiFi.channel()); 
  Serial.print("AP IP address: ");Serial.println(IP);
  Serial.print("MAC address: ");  Serial.println(WiFi.macAddress());
  
  server.begin();

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setHostname("HondaSlave");

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
// Telnet stream start
  TelnetStream.begin();
  Serial.println("ESPNow/Honda Slave Remote control VictronVenus V0.1");
  //Set device in AP mode to begin with
  initVariant();
  InitESPNow();
  addMasterAsPeer();

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
// Wifi status
    WiFi.printDiag(Serial);}

void sendData() {
    // Set values to send
  strcpy(myDataSlave.a, "Slave-->Master");
  myDataSlave.HondaRunning = gblHondaRunningFB;
  myDataSlave.HondaIgnitionOn = gblHondaIgnitionOn;
  myDataSlave.HondaStarting = gblHondaStarting;
  myDataSlave.HondaStartCount = gblSlaveHondaStartCount;
  const uint8_t *peer_addr = master.peer_addr;
  ESP_ERROR_CHECK(esp_now_send(peer_addr, (uint8_t *) &myDataSlave, sizeof(myDataSlave)));
}


// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  int indata= *data;
  memcpy(&myData, data, sizeof(myData));
  if (DEBUG) {
    Serial.println("Routine OnDataRecv...");
    Serial.print("  Last Packet Recv from: "); Serial.println(macStr);
    Serial.print("  Last Packet Recv Data: "); Serial.println(*data);
    Serial.println("");
    Serial.print("  Bytes received: ");
    Serial.println(data_len);
    Serial.print("  Char: ");
    Serial.println(myData.a);
    Serial.print("  HondaRunningFB: ");
    Serial.println(myData.HondaRunningFB);
    Serial.print("  HondaIgnitionOn: ");
    Serial.println(myData.HondaIgnitionOn);
    Serial.print("  HondaStart: ");
    Serial.println(myData.HondaStart);
    Serial.println();
  }
  if (NETDEBUG) {
    TelnetStream.println("Routine OnDataRecv...");
    TelnetStream.print("  Last Packet Recv from: "); TelnetStream.println(macStr);
    TelnetStream.print("  Last Packet Recv Data: "); TelnetStream.println(*data);
    TelnetStream.println("");
    TelnetStream.print("  Bytes received: ");
    TelnetStream.println(data_len);
    TelnetStream.print("  Char: ");
    TelnetStream.println(myData.a);
    TelnetStream.print("  HondaRunningFB: ");
    TelnetStream.println(myData.HondaRunningFB);
    TelnetStream.print("  HondaIgnitionOn: ");
    TelnetStream.println(myData.HondaIgnitionOn);
    TelnetStream.print("  HondaStart: ");
    TelnetStream.println(myData.HondaStart);
    TelnetStream.println();
  }
  gblHondaStartCommand = myData.HondaStart;
  HondaMain();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
 if (DEBUG) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
 }
 if (NETDEBUG) {
  TelnetStream.print("\r\nLast Packet Send Status:\t");
  TelnetStream.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
 }
}
void HondaMain()
{
  
  if(gblHondaStartCommand==true and gblHondaRunningFB==false){
    gblHondaStarting=true;
    HondaStart();
  }
  else if(gblHondaStartCommand==false and gblHondaRunningFB==true){
    HondaStop();
  }
  
}




 
void HondaStart()
{
  gblHondaStartCommand==false;
  if (DEBUG) {
      Serial.println("Routine HondaStart..");
      Serial.println("  Honda Ingition On..");
      Serial.println("  Honda delay 10 sec..");
  }
  digitalWrite(pinHondaIgnitionOn,LOW);
  delay(10000);
  if (DEBUG) {
      Serial.print("  Honda starting..");
  }
  digitalWrite(pinHondaStart,LOW);
  digitalWrite(LED,HIGH);
  delay(HondaCrankTime);
  digitalWrite(pinHondaStart,HIGH);
  digitalWrite(LED,LOW);
  if (DEBUG) {
      Serial.println("  Honda started..");
  }
  gblHondaStarting=false;
  gblSlaveHondaStartCount++;
  if(!HW_HondaFB) { 
    gblHondaRunningFB=true;
    }
}

  void HondaStop()
{
  if (DEBUG) {
      Serial.print("Honda stop..");
      Serial.println("Honda stopped");
  }
      digitalWrite(pinHondaIgnitionOn,HIGH);
      digitalWrite(pinHondaStart,HIGH);
      if(!HW_HondaFB) { gblHondaRunningFB=false;}
  }
void loop()
{
    ArduinoOTA.handle();
    WiFiClient client = server.available();   // Listen for incoming clients

//    Serial.println("loop slave..");
    if(millis() - startTime >=HondaSendStatusTime){
      startTime=millis();
      sendData();
    }

  if (client) {                             // If a new client connects,
  gpio_num_t pin13 = (gpio_num_t)(13 & 0x1F);
  gpio_num_t pin14 = (gpio_num_t)(13 & 0x1F);
  int state13=0;
  int state14=0;
  state13 = (GPIO_REG_READ(GPIO_OUT_REG)  >> pin13) & 1U;
  state14 = (GPIO_REG_READ(GPIO_OUT_REG)  >> pin14) & 1U;

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
            client.println("<p>GPIO 13 - State </p>");
            // If the output26State is off, it displays the ON button       
            if (state13==0) {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">ON</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 14 - State </p>");
            // If the state14 is off, it displays the ON button       
            if (state14==0) {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">ON</button></a></p>");
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

        Serial.print(" gblHondaStartCommand    :");Serial.println(gblHondaStartCommand);
        Serial.print(" gblHondaIgnitionOn      :");Serial.println(gblHondaIgnitionOn);
        Serial.print(" gblHondaStarting        :");Serial.println(gblHondaStarting);
        Serial.print(" gblHondaRunningFB       :");Serial.println(gblHondaRunningFB);     
        Serial.print(" gblSlaveHondaStartCount :");Serial.println(gblSlaveHondaStartCount);
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
      Serial.print(" AP IP address  : ");Serial.println(IP);
      Serial.print(" MAC address    : ");  Serial.println(WiFi.macAddress());
      Serial.print(" Channel        : "); Serial.println(WiFi.channel()); 
      Serial.print(" AP MAC         : "); Serial.println(WiFi.softAPmacAddress());
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
        TelnetStream.print(" ");TelnetStream.println(SWVERSION);

        TelnetStream.print(" gblHondaStartCommand      :");TelnetStream.println(gblHondaStartCommand);
        TelnetStream.print(" gblHondaIgnitionOn        :");TelnetStream.println(gblHondaIgnitionOn);
        TelnetStream.print(" gblHondaStarting          :");TelnetStream.println(gblHondaStarting);
        TelnetStream.print(" gblHondaRunningFB         :");TelnetStream.println(gblHondaRunningFB);     
        TelnetStream.print(" gblSlaveHondaStartCount   :");TelnetStream.println(gblSlaveHondaStartCount);
        TelnetStream.println(" Uptime : " + uptime_formatter::getUptime());
        TelnetStream.print(" Free mem                  :");TelnetStream.println(ESP.getFreeHeap());
       TelnetStream.print(" Debuglevel                 :");TelnetStream.println(NETDEBUG);
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
      TelnetStream.print(" AP IP address : ");TelnetStream.println(IP);
      TelnetStream.print(" MAC address   : ");  TelnetStream.println(WiFi.macAddress());
      TelnetStream.print(" Channel       : "); TelnetStream.println(WiFi.channel()); 
      TelnetStream.print(" AP MAC        : "); TelnetStream.println(WiFi.softAPmacAddress());
      break;
  }

}
 
