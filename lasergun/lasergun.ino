/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "Spooky";
const char * networkPswd = "***********";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.1.61";
const int udpPort = 8888;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


char *command = "fire";

void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  pinMode(0,INPUT);
  digitalWrite(0,HIGH);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop(){
  //only send data when connected
  if(connected){
    //Send a packet
    if (!digitalRead(0))
    {
      udp.beginPacket(udpAddress,udpPort);
      for (int i = 0; i < strlen(command); i++)
      {
        udp.write((uint8_t)command[i]);
        
      }
      udp.endPacket();
      Serial.println("Fire!");
        //Wait for 1/2 second
      delay(500);
    }
    
  }

}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}
