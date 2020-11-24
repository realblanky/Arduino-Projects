//importing ESP8266 Library & Adafruit MQTT Library 
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* Defining Pins and WLAN Network *********************************/
    //For RELAYS
#define D1 5    // GPIO5
#define D2 4    // GPIO4

    //FOR LEDS
#define D5 14     // GPIO14
#define D6 12     // GPIO12

    //Defining Wireless SSID and Password
#define WLAN_SSID "SSID" //Enter your SSID
#define WLAN_PASS "PASSWORD"  //Enter your Password

/************************* Adafruit.io Setup *********************************/
String  Request;
#define AIO_SERVER      "io.adafruit.com"     //Defining Adafruit's Server
#define AIO_SERVERPORT  1883    //Port 1883 for MQTT & 8883 for SSL
#define AIO_USERNAME  "USERNAME"    //Defining Adafruit Username
#define AIO_KEY  "KEY"     //Defining Adafruit Key
    // Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
    // SETTING UP MQTT CONNECTION
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
    // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Subscribe feed= Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/FEEDNAME");     //SET UP feed

/************************* SETTING DEFAULT STATE FOR SKETCH *********************************/
void MQTT_connect();
void setup()
{
  Serial.begin(115200);     //Upload speed to 115200
  Request = "";

  WiFi.disconnect();
  delay(3000);
  Serial.println("Initializing Connection to WIFI network");
  WiFi.begin(WLAN_SSID, WLAN_PASS);     //Attempts to connect WiFi Network
  while ((!(WiFi.status() == WL_CONNECTED))) 
  {
    delay(500);
    Serial.print(".");

  }
  Serial.println("WiFi successfully connected");    //Prints Local IP address
  Serial.println("Your IP is");
  Serial.println((WiFi.localIP().toString()));
      // Setup MQTT subscription for fbchat feed.
  mqtt.subscribe(&feed); //feed

  pinMode(D1, OUTPUT);    // SETTING PINMODE STATE FOR RELAY
  pinMode(D2, OUTPUT);


  pinMode(D5, OUTPUT);    // SETTING PINMODE STATE FOR LED
  pinMode(D6, OUTPUT);

  
  digitalWrite(D1, HIGH);   // SETTING INITIAL STATE FOR RELAY i.e HIGH for off
  digitalWrite(D2, HIGH);


  digitalWrite(D5, LOW);    // SETTING INITIAL STATE FOR LED i.e LOW for off
  digitalWrite(D6, LOW);

}

/************************* SETTING LOOP FUNCTION FOR SKETCH *********************************/
void loop()
{
      // Ensure the connection to the MQTT server is alive
 MQTT_connect();
 {
        // this is our 'wait for incoming subscription packets' busy subloop
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &feed) { //feed
        Request = ((char *)feed.lastread); //feed
        if (Request == "d1on") {      //If last feed matches with the character
          digitalWrite(D1, LOW);      //Relay state changes to LOW i.e ON
          digitalWrite(D5, HIGH);     //LED state changes to HIGH i.e ON
          Serial.println("IN1 & LED 1 is ON!");
        }
        if (Request == "d1off") {     //If last feed matches with the character
          digitalWrite(D1, HIGH);     //Relay state changes to HIGH i.e OFF
          digitalWrite(D5, LOW);      //LED state changes to HIGH i.e OFF
          Serial.println("IN1 & LED 1 is OFF!");
        }

        if (Request == "d2on") {
          digitalWrite(D2, LOW);
          digitalWrite(D6, HIGH);
          Serial.println("IN2 & LED 2 is ON!");
        }
        if (Request == "d2off") {
          digitalWrite(D2, HIGH);
          digitalWrite(D6, LOW);
          Serial.println("IN2 & LED 2 is OFF!");
        }

      }

    }

  }

}
/************************* Connection to MQTT server *********************************/
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care of connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
