# ESP32HondaRemoteStart
Wireless remote start of Honda Generator using ESP32-pair

Two ESP32, HondaMaster and HondaSlave
Use ESPNow with fixed MAC addresses in code to pair the two ESP32's
Can use any MAC address , ESP32 will use the entered MAC addess
Example from web used
ESP32 / ESPNow:   https://www.instructables.com/ESP32-With-ESP-Now-Protocol/
Fixed MAC address:  https://randomnerdtutorials.com/get-change-esp32-esp8266-mac-address-arduino/
(Does NOT use Wifi AP to discover ESPNow - slave)

HondaMaster:

Intput pins
Pin 15 : start aggregate from Victron Venus
Pin 14 : Manual switch , manual start
Output Pins
Pin 13:  Feedback from Honda, running,  LED


HondaSlave
Output Pins
Pin 14 :  Honda Ignition ON to relay
Pin 13 : Honda crank (start) to relay


