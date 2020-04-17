/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#include <Arduino.h>
#include <analogWrite.h>
#include <Servo.h>

#include <PID_v1.h>
#include <WiFi.h>

//--- SETTINGS ------------------------------------------------
const char* ssid = "bbox2-5de4";         // enter the name (SSID) of your WIFI network
const char* password = "ARLARLARL";         // enter your WIFI network PASSWORD
WiFiServer server(8000);                   // Default Virtuino Server port
IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace with your router IP
//---

//---VirtuinoCM  Library settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
//---


boolean debug = false;


#define in1 32
#define in2 33
#define enA 25
#define in3 13
#define in4 12
#define enB 26
#define enc1 22
#define enc2 23


//Define Variables we'll be connecting to
double Setpoint, PVin, OUT, OUTMot;

//Specify the links and initial tuning parameters
double Kp = 0.05, Ki = 0.2, Kd = 0;
PID myPID(&PVin, &OUT, &Setpoint, Kp, Ki, Kd, DIRECT);


hw_timer_t * timer = NULL;

const int freq = 5000;
const int enAChannel = 0;
const int enAresolution = 8;
const int enBChannel = 1;
const int enBresolution = 8;


float Vitaux = 0;
float Vit = 0;
float compteur1 = 0;
float compteur2 = 0;

int trig = 14;
int echo = 34;
long lecture_echo;
long cm, cmVal;
int MemoSP = 0;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;

int T[10];
int TDN[10];
boolean CptT[10];

//------------------------------------------------------
void IRAM_ATTR isr1() {
  compteur1 = compteur1 + 1;
}
void IRAM_ATTR isr2() {
  compteur2 = compteur2 + 1;
}
void IRAM_ATTR onTimer() {
  T[1] =  T[1] + 1.0;
  T[2] =  T[2] + 1.0;
}
//------------------------------------------------------
void setup() {
  Serial.begin(115200);



  myPID.SetMode(AUTOMATIC);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered
  timerAlarmWrite(timer, 1000000, true); // 1000000 * 1 us = 1/10 s, autoreload true
  timerAlarmEnable(timer); // enable

  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);
  //attachInterrupt(enc1, isr1, HIGH);
  //attachInterrupt(enc2, isr2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  ledcSetup(enAChannel, freq, enAresolution);
  ledcAttachPin(enA, enAChannel);
  ledcSetup(enBChannel, freq, enBresolution);
  ledcAttachPin(enB, enBChannel);

  virtuino.begin(onReceived, onRequested, 256); //Start Virtuino. Set the buffer to 512. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
  server.begin();

}

void loop() {

  virtuinoRun();

  Tempos1();
  Tempos2();

    V[1] = T[2];

  if (TDN[2]& MemoSP == 0) {
    Setpoint = 900;//mm/sec
    MemoSP = 1;

  } else if (TDN[2]& MemoSP == 1) {
    Setpoint = 500;//mm/sec
    MemoSP = 0;

  }

  PVin = Vit;
  

  myPID.Compute();

  MesDistance();

  OUTMot = V[3];//OUT;
  V[0] = OUTMot;
  if (OUT < 100) OUTMot = 100;
  if (OUT > 250)OUTMot = 250;

  ledcWrite(enAChannel, OUTMot);
  ledcWrite(enBChannel, OUTMot);


  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);


  if (TDN[1]) {
    //calcul vitesse mm / secondes
    Vitaux = ((compteur1 * (83.25 / 20.0)));
    Vit = ((((Vit * 20.0) + Vitaux) / 21.0));
    compteur1 = 0;
  }
  //Serial.print("PVin: ");
  // Serial.println(Vit);
  //Serial.print('\n');
  //Serial.println(TDN[1]);

  // Serial.print(" OUT: ");
  //Serial.println(OUT);

  //vDelay(100);
}

//*****Serial.print(compteur1);****************
void MesDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  lecture_echo = pulseIn(echo, HIGH);
  cmVal = lecture_echo / 29;
  cm = ((cm * 20) + cmVal ) / (20 + 1);
}

//++++++++++++++++++++++++++++++++++++++++++

//Gestion temporisation
void Tempos1() {
  if (!TDN[1]) {
    CptT[1] = true;
  }
  else {
    T[1] =  0.0;
    TDN[1] =  0.0;
    CptT[1] = false;
  }
  if (T[1] >= 1) { // présélection de 600 donc 60 secondes
    TDN[1] =  1.0;   // bit down
    T[1] = 1;
  }
}
void Tempos2() {
  if (!TDN[2]) {
    CptT[2] = true;
  }
  else {
    T[2] =  0.0;
    TDN[2] =  0.0;
    CptT[2] = false;
  }
  if (T[2] >= 30) { // présélection de 600 donc 60 secondes
    TDN[2] =  1.0;   // bit down
    T[2] = 30;
  }

  //vDelay(1000);

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
