/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#include <Arduino.h>
#include <WiFi.h>
#include <Servo.h>
#include "MegunoLink.h"


//----------I2C--------------
#include <Wire.h>
int error = 0;

//----- MEGUOLINK-------------
// Millis valu
// Interval (milliseconds) between sending analog data
const unsigned SendInterval = 200; // [ms]
// The plot we are sending data to. A TimePlot is used here
TimePlot MyPlot;
long LastSent;


// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output15State = "off";
String output4State = "off";
// Assign output variables to GPIO pins
const int output15 = 15;
const int output4 = 4;

//--- SETTINGS ------------------------------------------------
const char* ssid = "bbox2-5de4";         // enter the name (SSID) of your WIFI network
const char* password = "ARLARLARL";         // enter your WIFI network PASSWORD
WiFiServer server1(6100);                   // Default Virtuino server1 port
WiFiServer server2(80);                   // Default Virtuino server1 port
IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace with your router IP
#include "VirtuinoCM.h"
VirtuinoCM virtuino;
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
boolean debug = false;
//-------------------------------------------------------------
int T[10];
int TDN[10];
boolean Cpt[10];
int Cond[10];
int c;
hw_timer_t * timer = NULL;

//--Moteur PWM---------
#define in1 32
#define in2 33
#define enA 25
#define in3 13
#define in4 12
#define enB 26


const int freq = 5000;
const int enAChannel = 4;
const int enAresolution = 8;
const int enBChannel = 5;
const int enBresolution = 8;
int OUTMot = 0;

//----Mesure de vitesse cpt impulsions
#define enc1 22
#define enc2 23

float Vitaux = 0;
float Vit = 0;
float compteur1 = 0;
float compteur2 = 0;

//-------------------MESURE DE DISTANCE--------------
int trig = 14;
int echo = 34;
long lecture_echo;
long cm, cmVal;
int MemoSP = 0;


//---------------------VOID IRQ CODEUR MESURE DE VITESSE----
void IRAM_ATTR isr1() {
  compteur1 = compteur1 + 1;
}
void IRAM_ATTR isr2() {
  compteur2 = compteur2 + 1;
}
//---------------------VOID IRQ TEMPO----------------------------------------
void IRAM_ATTR onTimer() {
  if (Cpt[1]) {
    T[1] =  T[1] + 1.0;
  }
  if (Cpt[2]) {
    T[2] =  T[2] + 1.0;
  }
  if (Cpt[3]) {
    T[3] =  T[3] + 1.0;
  }
  if (Cpt[4]) {
    T[4] =  T[4] + 1.0;
  }
}
//--------------SERVO---------------
static const int servoPin1 = 18;
Servo servo1;
static const int servoPin2 = 19;
Servo servo2;
int PWMServo = 0;



//*********************SETUP********************************
void setup() {
  Serial.begin(9600);

  //-----------I2C--------------
  Wire.begin();                // join i2c bus with address #8


  //-----------MEGUOLINK
  LastSent = millis();

  MyPlot.SetTitle("My Analog Measurement");
  MyPlot.SetXLabel("Time");
  MyPlot.SetYLabel("Value");
  MyPlot.SetSeriesProperties("ADCValue", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);

  //----VIRTUINO-----
  virtuino.begin(onReceived, onRequested, 256); //Start Virtuino. Set the buffer to 512. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library
  connectToWiFiNetwork();
  server1.begin();
  server2.begin();

  //-----IRQ TEMPO-----
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered
  timerAlarmWrite(timer, 1000, true); // 1000000 * 1 us = 1/10 s, autoreload true
  timerAlarmEnable(timer); // enable

  //----Moteur PWM------
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(15, OUTPUT);


  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);


  ledcSetup(enAChannel, freq, enAresolution);
  ledcAttachPin(enA, enAChannel);
  ledcSetup(enBChannel, freq, enBresolution);
  ledcAttachPin(enB, enBChannel);


  //-----------MESURE VITESSE-------ATTENTION TROP RAPIDE------A FAIRE FAIRE PAR UN AUTRE CPU---
  //pinMode(enc1, INPUT_PULLUP);
  //pinMode(enc2, INPUT_PULLUP);
  //attachInterrupt(enc1, isr1, HIGH);
  //attachInterrupt(enc2, isr2, HIGH);

  //------------MESURE DISTANCE------------
  int trig = 14;
  int echo = 34;
  long lecture_echo;
  long cm, cmVal;
  int MemoSP = 0;

  //--------SERVO-----
  servo1.attach(18);
  servo2.attach(19);

  //--------Init Variables------

}




//*********************LOOP********************************
void loop() {


  //-------------I2C-------------------
  Wire.requestFrom(8, 1);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    c = Wire.read(); // receive a byte as character
    // Serial.println(c);         // print the character
  }
  //------Web pc-----------
  // clientWeb();

  //---VIRTUINO---

  virtuinoRun();
  clientWeb();
  //-----------MESURE DE DISTANCE-----
  MesDistance();

  //-----Exemple de Tempo
  //-----T[1]
  Cond[1] = !TDN[1];
  TDN[1] = Tempos( 1, 1);
  if (TDN[1]) {

  }
  //----T[2]
  Cond[2] = !TDN[2];
  TDN[2] = Tempos( 2, 10);
  if (TDN[2]) {

  }

  //-----Vitesse roues-----
  //if (!digitalRead(15)) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //} else {
  //    digitalWrite(in1, LOW);
  //    digitalWrite(in2, LOW);
  //    digitalWrite(in3, LOW);
  //    digitalWrite(in4, LOW);
  //}
  OUTMot = V[3];
  ledcWrite(enAChannel, OUTMot);
  ledcWrite(enBChannel, OUTMot);


  //-------------MESURE DISTANCE---------
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);

  //-----------SERVO----------
  if (!TDN[3] & (T[4] == 0)) {
    Cond[3] = 1;
  } else {
    Cond[3] = 0;
  }
  TDN[3] = Tempos( 3, 1500);

  if (TDN[3]) {
    PWMServo = 1;
    Cond[4] = 1;

  }
  TDN[4] = Tempos( 4, 2000);
  if (TDN[4]) {
    PWMServo = 0;
    Cond[4] = 0;

  }

  //if (PWMServo == 0) servo1.write(int(V[7]));
  //if (PWMServo == 0) servo1.write(int(V[8]));
  //if (PWMServo == 0) servo2.write(int(V[7]));
  //if (PWMServo == 0) servo2.write(int(V[8]));


  //------------meguolink

  if ((millis() - LastSent) > SendInterval)
  {
    LastSent = millis();
    MyPlot.SendData("Distance", c);
    //MyPlot.SendData("Vitesse", OUTMot);
  }


  //-----VIRTUINO---------------------------
  //vDelay(100);
}
//---FIN LOOP----





//-----Gestion temporisationS------
int Tempos( int NUM, int PRESEL) {
  if (Cond[NUM]) {
    Cpt[NUM] = true;
  }
  else {
    T[NUM] =  0.0;
    TDN[NUM] =  0.0;
    Cpt[NUM] = false;
  }
  if (T[NUM] >= PRESEL) { // présélection de 600 donc 60 secondes
    TDN[NUM] =  1.0;   // bit down
    T[NUM] = PRESEL;
  }
  return TDN[NUM];
}

//------MESURE DE DISTANCE-----------
void MesDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  lecture_echo = pulseIn(echo, HIGH);
  cmVal = lecture_echo / 29;
  cm = ((cm * 100) + cmVal ) / (100 + 1);
  V[6] = cm;
}

//============================================================== connectToWiFiNetwork
void connectToWiFiNetwork() {
  Serial.println("Connecting to " + String(ssid));
  // If you don't want to config IP manually disable the next two lines
  IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
  WiFi.mode(WIFI_STA);                       // Config module as station only.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
}


//============================================================== onCommandReceived
/* This function is called every time Virtuino app sends a request to server1 to change a Pin value
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
  WiFiClient client = server1.available();
  if (!client) return;
  if (debug) Serial.println("Connected");
  unsigned long timeout = millis() + 3000;
  while (!client.available() && millis() < timeout) delay(1);
  if (millis() > timeout) {
    Serial.println("timeout");
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

//********************************************
void clientWeb() {
  WiFiClient client = server2.available(); // Listen for incoming clients
  if (client) { // If a new client connects,
    Serial.println("New Client."); // print a message out in the serial port
    String currentLine = ""; // make a String to hold incoming data from the client
    while (client.connected()) { // loop while the client's
      if (client.available()) { // if there's bytes to read

        char c = client.read(); // read a byte, then
        Serial.write(c); // print it out the serial

        header += c;
        if (c == '\n') { // if the byte is a newline
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println("Refresh: 5");
            client.println();
            // turns the GPIOs on and off
            if (header.indexOf("GET /15/on") >= 0) {
              Serial.println("GPIO 15 on");
              output15State = "on";
              digitalWrite(output15, HIGH);
            } else if (header.indexOf("GET /15/off") >= 0) {
              Serial.println("GPIO 15 off");
              output15State = "off";
              digitalWrite(output15, LOW);
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              output4State = "on";
              digitalWrite(output4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              output4State = "off";
              digitalWrite(output4, LOW);
            }
            // Display the HTML web page

            client.println("<!DOCTYPE html><html>");

            client.println("<head><meta name=\"viewport\"content=\"width=device-width, initial-scale=1\">");

            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border:none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color:#555555;}</style></head>");


            client.println("<body><h1>ESP32 Web server1</h1>");
            // Display current state, and ON/OFF buttons for GPIO 15
            client.println("<p>GPIO 15 - State " + output15State + " " + V[3] + "</p>");
            // If the output15State is off, it displays the ON button
            if (output15State == "off") {
              client.println("<p><a href=\"/15/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/15/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            // Display current state, and ON/OFF buttons for GPIO 4
            client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // If the output2State is off, it displays the ON

            if (output4State == "off") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            client.println("</body></html>");


            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') { // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
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
}
