/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#include <Arduino.h>
#include <WiFi.h>

//--- SETTINGS ------------------------------------------------
const char* ssid = "bbox2-5de4";         // enter the name (SSID) of your WIFI network
const char* password = "ARLARLARL";         // enter your WIFI network PASSWORD
WiFiServer server(8000);                   // Default Virtuino Server port
IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace with your router IP
#include "VirtuinoCM.h"
VirtuinoCM virtuino;
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
boolean debug = false;
//-------------------------------------------------------------






//*****************************************************
void setup() {
  Serial.begin(115200);

  virtuino.begin(onReceived, onRequested, 256); //Start Virtuino. Set the buffer to 512. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
  server.begin();
  V[0] = 0;
    V[1] = 0;
}

//*****************************************************
void loop() {

  virtuinoRun();
  V[0] = V[0] + 0.001;
  if (V[0]>100000) V[0]=0;
  V[1] = V[3];
  //vDelay(100);
}


//============================================================== connectToWiFiNetwork
void connectToWiFiNetwork() {
  //Serial.println("Connecting to " + String(ssid));
  // If you don't want to config IP manually disable the next two lines
  IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
  WiFi.mode(WIFI_STA);                       // Config module as station only.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  // Serial.println("");
  //Serial.println("WiFi connected");
  //Serial.println(WiFi.localIP());
}


//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
   The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
   The 'variableIndex' is the pin number index of Virtuino app
   The 'valueAsText' is the value that has sent from the app   */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
    if (variableIndex < V_memory_count) V[variableIndex] = value;          // copy the received value to arduino V memory array
  }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex) {
  if (variableType == 'V') {
    if (variableIndex < V_memory_count) return  String(V[variableIndex]); // return the value of the arduino V memory array
  }
  return "";
}


//==============================================================
void virtuinoRun() {
  WiFiClient client = server.available();
  if (!client) return;
  if (debug) Serial.println("Connected");
  unsigned long timeout = millis() + 3000;
  while (!client.available() && millis() < timeout) delay(1);
  if (millis() > timeout) {
    if (debug) Serial.println("timeout");
    client.flush();
    client.stop();
    return;
  }
  virtuino.readBuffer = "";  // clear Virtuino input buffer. The inputBuffer stores the incoming characters
  while (client.available() > 0) {
    char c = client.read();         // read the incoming data
    virtuino.readBuffer += c;       // add the incoming character to Virtuino input buffer
    if (debug) Serial.write(c);
  }
  client.flush();
  if (debug) Serial.println("\nReceived data: " + virtuino.readBuffer);
  String* response = virtuino.getResponse();   // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
  if (debug) Serial.println("Response : " + *response);
  client.print(*response);
  client.flush();
  delay(10);
  client.stop();
  if (debug) Serial.println("Disconnected");
}


//============================================================== vDelay
void vDelay(int delayInMillis) {
  long t = millis() + delayInMillis;
  while (millis() < t) virtuinoRun();
}
