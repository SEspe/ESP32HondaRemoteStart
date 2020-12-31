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
#define WIFI false

#define WIFI_CHANNEL 5
 
// Onboard ledpin

#define LED 4 
#define pinHondaStart 13
#define pinHondaIgnitionOn 14
#define pinHondaRunningFB 15
#define HW_HondaFB  0  // HW feedback enabled
#define DEBUG 1
#define HondaCrankTime 3000   // ms crank time
#define HondaSendStatusTime 10000   // ms Slave send interval
unsigned long startTime=0;


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
} struct_slavemessage;

// Create a struct_message called myData for master message to slave
struct_slavemessage myDataSlave;


bool gblHondaRunningFB= false;
bool gblHondaStarting= false;
bool gblHondaStartCommand= false;
bool gblHondaIgnitionOn = false;
//  Set legal MAC address; observe success on setting MAC in serioal monitor
const uint8_t slaveCustomMac[] = {0x30, 0xAE, 0xA4, 0x1A, 0xAE, 0x36};  //Custom mac address for This Slave Device
const uint8_t masterCustomMac[] = {0x30, 0xAE, 0xA4, 0x89, 0x92, 0x71}; //Custom mac address for The Master Device 
esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
const byte maxDataFrameSize = 200;
uint8_t dataToSend[maxDataFrameSize];
byte cnt=0;
esp_err_t sendResult;
/* -----------------Declarations-----------------*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

//
//  Set fixed MAC address and set channel to value WIFI_CHANNEL
//  Checks status with ESP_ERROR_CHECK 
//
void initVariant()
{
  uint8_t primaryChan;
  wifi_second_chan_t secondChan;
  WiFi.mode(WIFI_MODE_STA);                                                             // WIFI_STA Mode
// Set fixed MAC
  ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, &slaveCustomMac[0])); // #include <esp_wifi.h> required to compile this line
// Set channel
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  Serial.print("get_channel Status: "); Serial.println(primaryChan);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL,secondChan));
  ESP_ERROR_CHECK(esp_wifi_get_channel(&primaryChan,&secondChan));
  Serial.print("get_channel Status: "); Serial.println(primaryChan);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));

  

      const uint8_t protocol = WIFI_PROTOCOL_LR;
      ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, protocol) );
}

void addMasterAsPeer()
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           masterCustomMac[0], masterCustomMac[1], masterCustomMac[2], masterCustomMac[3], masterCustomMac[4], masterCustomMac[5]);
  memcpy( &master.peer_addr, &masterCustomMac, 6 );
  master.channel = WIFI_CHANNEL;
  master.encrypt = 0;
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



void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Honda Slave Remote control VictronVenus V0.1");
  //Set device in AP mode to begin with
  initVariant();
  InitESPNow();
  addMasterAsPeer();

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  WiFi.printDiag(Serial);
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
}

void sendData() {
    // Set values to send
  strcpy(myDataSlave.a, "Slave-->Master");
  myDataSlave.HondaRunning = gblHondaRunningFB;
  myDataSlave.HondaIgnitionOn = gblHondaIgnitionOn;
  myDataSlave.HondaStarting = gblHondaStarting;
  const uint8_t *peer_addr = master.peer_addr;
  ESP_ERROR_CHECK(esp_now_send(peer_addr, (uint8_t *) &myDataSlave, sizeof(myDataSlave)));
}


// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
  int indata= *data;
  Serial.print("Decoded data: "); Serial.println(indata);
  memcpy(&myData, data, sizeof(myData));
  #ifdef DEBUG
  Serial.print("Bytes received: ");
  Serial.println(data_len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("HondaRunningFB: ");
  Serial.println(myData.HondaRunningFB);
  Serial.print("HondaIgnitionOn: ");
  Serial.println(myData.HondaIgnitionOn);
  Serial.print("HondaStart: ");
  Serial.println(myData.HondaStart);
  Serial.println();
  #endif
  gblHondaStartCommand = myData.HondaStart;
  HondaMain();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void HondaMain()
{
  
  if(gblHondaStartCommand==true){
    gblHondaStarting=true;
    HondaStart();
  }
  else {HondaStop();}
  }
void HondaStart()
{
  gblHondaStartCommand==false;
  Serial.println("Honda Ingition On..");
  digitalWrite(pinHondaIgnitionOn,LOW);
  Serial.println("Honda delay 10 sec..");
  delay(10000);
  Serial.print("Honda starting..");
  digitalWrite(pinHondaStart,LOW);
  digitalWrite(LED,HIGH);
  delay(HondaCrankTime);
  digitalWrite(pinHondaStart,HIGH);
  digitalWrite(LED,LOW);
  Serial.println("Honda started..");
  gblHondaStarting=false;
  if(!HW_HondaFB) { gblHondaRunningFB=true;}
  
  
  }

  void HondaStop()
{
      Serial.print("Honda stop..");
      digitalWrite(pinHondaIgnitionOn,HIGH);
      digitalWrite(pinHondaStart,HIGH);
      
      Serial.println("Honda stopped");
      if(!HW_HondaFB) { gblHondaRunningFB=false;}
  }
void
loop(void)
{

//    Serial.println("loop slave..");
    if(millis() - startTime >=HondaSendStatusTime){
      startTime=millis();
      sendData();
    }
  
}
 
