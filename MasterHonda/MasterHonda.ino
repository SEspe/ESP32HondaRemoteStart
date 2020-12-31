/**
 * 
 * Sjekkesp_wifi_init
 * 
 * 
   ESPNOW - Honda Remote Start using ESPNOW - Master
   Date: 9th July 2020
   Author: Stein Espe
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32 to control a Hona EU70IS
   Description: This sketch consists of the code for the Master module.
   Communication between Master and Slave use ESPNow-protocol, 
   fixed MAC eddresses entrered in code
   
   Code based on example from this source:
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

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
*/

#define DEBUG 1                         // Include this definition to compile & show DEBUGmessages
#define WIFI false
//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

//bool isPaired                           // ESP Now Pair status


bool isPaired;
#define WIFI_CHANNEL 5
#define PRINTSCANRESULTS false
#define DELETEBEFOREPAIR false
const uint8_t slaveCustomMac[] = {0x30, 0xAE, 0xA4, 0x1A, 0xAE, 0x36};  //Custom mac address for This Slave Device
const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x89, 0x92, 0x71}; //Custom mac address for The Master Device 

const uint8_t maxDataFrameSize=200;
// Global copy of slave
esp_now_peer_info_t slave;
const esp_now_peer_info_t *slaveNode = &slave;
/*-------------------Declarations-------------------*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);


//  Input pin
#define pinHondaStartedFB 13                // pin for Honda StartedFB (LED)
#define pinHondaStartedFBOnboard 2          // pin for Honda StartedFB (LED)
#define pinHondaStart 15                    // pin for Honda Start/Crank
#define pinHondaManualStart 14              // pin for Honda Manual Start
bool gblSlavelHondaStart = false ;          // default global, HondaStart=false = off.  Will contain Slave's reported status
bool gbllocHondaStart = false;
#define HondaRestartTime 30000   // ms Restart blocking time
unsigned long startTime=0;

// Debounce var's
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros=micros();

// Wifi and slave status
bool slaveFound = 0;
bool connected = 0;


// Structure example to send data
// Must match the receiver structure
typedef struct struct_mastermessage {
  char a[32];
  bool HondaRunningFB;
  bool HondaIgnitionOn;
  bool HondaStart;
} struct_mastermessage;

// Create a struct_message called myData for master message to slave
struct_mastermessage myData;

// Structure example to receive data
// Must match the receiver structure
typedef struct struct_slavemessage {
  char a[32];
  bool HondaIgnitionOn;
  bool HondaStarting;
  bool HondaRunning;
} struct_slavemessage;

// Create a struct_message called myData for master message to slave
struct_slavemessage myDataSlave;

// Set fixed mac & select channel
// 
void initVariant()
{
  uint8_t primaryChan;
  wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);      // enable WARN logs from WiFi stack
 
  Serial.println("InitVariant"); // esp_base_mac_addr_set
  WiFi.mode(WIFI_MODE_STA); // WIFI_STA Mode
  ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, &masterCustomMac[0]));   // #include <esp_wifi.h> required to compile this line

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
void addSlaveAsPeer()
{
  memcpy( &slave.peer_addr, &slaveCustomMac, 6 );
  slave.channel = WIFI_CHANNEL;
  slave.encrypt = 0;
  if( esp_now_add_peer(slaveNode) == ESP_OK)
  {
    Serial.println("Slave Added As Peer!");
  }
}
//
// Init ESP Now with fallback
//
void InitESPNow() {
  
  WiFi.mode(WIFI_STA);
  Serial.print("Mac this station:");Serial.println(WiFi.macAddress() );
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    esp_wifi_set_ps(WIFI_PS_NONE);
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}






// send data
void sendData() {
    // Set values to send
  strcpy(myData.a, "Master-->Slave");
  myData.HondaRunningFB = false;
  myData.HondaIgnitionOn = digitalRead(pinHondaManualStart);
  myData.HondaStart = gbllocHondaStart;
//  gbllocHondaStart =  !digitalRead(pinHondaStart);
  Serial.print("HondaStart: ");
  Serial.println(myData.HondaStart);
  const uint8_t *peer_addr = slave.peer_addr;

  ESP_ERROR_CHECK(esp_now_send(peer_addr, (uint8_t *) &myData, sizeof(myData)));

}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  #ifdef DEBUG
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
}

// callback when data is recv from Slave
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
  int indata= *data;
  Serial.print("Decoded data: "); Serial.println(indata);
  memcpy(&myDataSlave, data, sizeof(myDataSlave));
  gblSlavelHondaStart=myDataSlave.HondaRunning;
  #ifdef DEBUG
     Serial.print("gbllocHondaStartReceived:");
     Serial.println(gbllocHondaStart);
     Serial.print("gblSlavelHondaStartReceived:");
     Serial.println(gblSlavelHondaStart);
  #endif
  #ifdef DEBUG
  Serial.print("Bytes received: ");
  Serial.println(data_len);
  Serial.print("Char: ");
  Serial.println(myDataSlave.a);
  Serial.print("HondaRunning: ");
  Serial.println(myDataSlave.HondaRunning);
  Serial.print("HondaIgnitionOn: ");
  Serial.println(myDataSlave.HondaIgnitionOn);
  Serial.print("HondaStarting: ");
  Serial.println(myDataSlave.HondaStarting);
  Serial.println();
  #endif
  if(gblSlavelHondaStart) {
     digitalWrite(pinHondaStartedFB,HIGH);   
     digitalWrite(pinHondaStartedFBOnboard,HIGH);   
  }
  if(!gblSlavelHondaStart) {
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


gbllocHondaStart = (digitalRead(pinHondaManualStart) or !digitalRead(pinHondaStart));

}



void setup() {
  // Define SerialSpeed
  Serial.begin(115200);
  Serial.println("ESPNow/Honda Master Remote control VictronVenus V0.1");
  // print info

  Serial.print("Honda start input pin on :");
  Serial.println(pinHondaStart);
  Serial.print("Honda ignition on pin on :");
  Serial.println(pinHondaManualStart);  
  Serial.println("ESPNow/Honda Master control connected to VictronVenus V0.0");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  initVariant();
  InitESPNow();

  addSlaveAsPeer();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  // Set pin mode & attach interrupt
  pinMode(pinHondaStart,INPUT);  // Honda  Start / Stopp fra Victron Venus rele 0
  attachInterrupt(pinHondaStart, intr_HondaStart, CHANGE);
  gbllocHondaStart = !digitalRead(pinHondaStart);
 pinMode(pinHondaManualStart,INPUT_PULLUP);  // Honda ingnition ON 
 attachInterrupt(pinHondaManualStart, intr_HondaManualStart, CHANGE);
 pinMode(pinHondaStartedFB,OUTPUT);  // Honda startedFB 
 pinMode(pinHondaStartedFBOnboard,OUTPUT);  // Honda startedFB   
 digitalWrite(pinHondaStartedFB,LOW);
 digitalWrite(pinHondaStartedFBOnboard,LOW);

  
  }





void loop() {
// Check for change in input;  done in loop to avoid ripple, and evaluate legality of start/stop
// Rules
//  Stop  :  Always legal
//  Start :   Legal when not  gbllocHondaStart ( Master's knowledge of status) 
//            and
//            Not blocked by rapis start requests (time in HondaRestartTime) 
  delay(3000);
  #ifdef DEBUG
  Serial.print(" gbllocHondaStart :");Serial.println(gbllocHondaStart);
  Serial.print(" gblSlavelHondaStart :");Serial.println(gblSlavelHondaStart);
  #endif

  if((gbllocHondaStart !=gblSlavelHondaStart) &&  (millis() - startTime >=HondaRestartTime)){
//    gbllocHondaStart=gblSlavelHondaStart;
    startTime=millis();

    sendData();
    delay(200);
    sendData();
    
  }
  #ifdef DEBUG
  Serial.print(" Pin pinHondaStart :");Serial.println(!digitalRead(pinHondaStart));
  Serial.print(" Pin pinHondaStartedFB :");Serial.println(digitalRead(pinHondaStartedFB));
  Serial.print(" Pin pinHondaManualStart :");Serial.println(digitalRead(pinHondaManualStart));
  #endif
  //  delay(2000);
//  Serial.println("loop Master..");
//  sendData();
}
