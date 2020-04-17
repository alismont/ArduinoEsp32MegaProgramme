/*
  Alain Lismont

*/

#include <Arduino.h>
#include "MegunoLink.h"
#include "Wire.h"
#include "TimerOne.h"


//--------VARIABLES POT----------------
int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor


//---------VARIABLES GIRO--------------
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO




//-----VARIABLES MEGUOLINK-------------
// Millis value when the data was last sent.
long LastSent;
// Interval (milliseconds) between sending analog data
const unsigned SendInterval = 10; // [ms]
// The plot we are sending data to. A TimePlot is used here
TimePlot MyPlot;

//---------------VARIABLES BOUSSOLE-----------
#define addr 0x1E //I2C Address for The HMC5883

//---------------VARIABLE BOUTON---------------
#define BP A3
int BPVal = 0;

//-----------VARIABLE MOTEURS------------
#define M1CCW 2
#define M1CW 5
#define M2CCW 6
#define M2CW 8




//---------------VARIABLES ESSAI FILTRE---------------
//--1er filtre
float AverageMesure = 0;
int MeasurementsToAverage = 20;

//--2em filtre
const int RunningAverageCount = 16;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;


//--3em filtre
#include "Filter.h"

// the <float> makes a filter for float numbers
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
ExponentialFilter<float> FilteredTemperature(20, 0);

//------------------------VARIABLES TEMPO  IRQ-------------------------------------
int T[10];
int TDN[10];
boolean Cpt[10];
int Cond[10];
int c;


//----VARIABLES Mesure de vitesse cpt impulsions----------------------------------
//----------------ENCODEUR------------------
#define M1CCWENC 15
int M1CCWENCIN = 0;
#define M1CWENC A8
int M1CWENCIN = 0;

#define M2CCWENC 18
int M2CCWENCIN = 0;
#define M2CWENC 19
int M2CWENCIN = 0;

float Vitaux1 = 0;
float Vit1 = 0;
float Vitaux2 = 0;
float Vit2 = 0;
volatile float compteur1 = 0;
volatile float compteur2 = 0;


//---------------------VOID IRQ CODEUR MESURE DE VITESSE----
void INTer1() {
  noInterrupts();
  compteur1 += 1;

  interrupts();
}
void INTer2() {
  noInterrupts();
  compteur2 += 1;
  interrupts();
}
//---------------------VOID IRQ TEMPO----------------------------------------
void callback() {

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

//----------------BLUETOOTH---------------
#include "VirtuinoBluetooth.h"                           // Include VirtuinoBluetooth library to your code
//#include <SoftwareSerial.h>                              //  Disable this line if you want to use hardware serial
//SoftwareSerial bluetoothSerial =  SoftwareSerial(2,3);   // arduino RX pin=2  arduino TX pin=3    connect the arduino RX pin to bluetooth module TX pin   -  connect the arduino TX pin to bluetooth module RX pin.  Disable this line if you want to use hardware serial
VirtuinoBluetooth virtuino(Serial2, 9600);




//******************SETUP*****************************
void setup() {

  Serial.begin(115200);

  //---------VIRTUINO BLUTOOTH------------------
  virtuino.DEBUG = false;


  //------------I2C---------------------
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  accelgyro.initialize();

  //------------------INIT IRQ TEMPO------------------------------
  Timer1.initialize(100000);         // initialize timer1
  //Timer1.setPeriod(100);
  Timer1.attachInterrupt(callback);


  //----------BOUSOLE----------------
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();

  //----------MOTEURS---------------
  pinMode(M1CCW, OUTPUT);
  pinMode(M1CW, OUTPUT);
  pinMode(M2CCW, OUTPUT);
  pinMode(M2CW, OUTPUT);
  pinMode(ledPin, OUTPUT);

  //----------ENCODEUR--------
  //-----------MESURE VITESSE---------
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), INTer1, HIGH );
  attachInterrupt(digitalPinToInterrupt(19), INTer2, HIGH );


  //-----------MEGUOLINK
  //LastSent = millis();

  MyPlot.SetTitle("My Analog Measurement");
  MyPlot.SetXLabel("Time");
  MyPlot.SetYLabel("Value");
  //  MyPlot.SetSeriesProperties("ax", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("ay", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("az", Plot::Green, Plot::Solid, 1, Plot::NoMarker);

  //
  //  MyPlot.SetSeriesProperties("x", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("y", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("z", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  //
  //  MyPlot.SetSeriesProperties("BPVal", Plot::Green, Plot::Solid, 1, Plot::NoMarker);

  // MyPlot.SetSeriesProperties("M1CWENCIN", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  //MyPlot.SetSeriesProperties("M1CCWENCIN", Plot::Red, Plot::Solid, 1, Plot::NoMarker);

  //MyPlot.SetSeriesProperties("M1CWENCIN", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("Vit1", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("Vit2", Plot::Blue, Plot::Solid, 1, Plot::NoMarker);

  // interrupts();
}





//*******************LOOP******************************
void loop() {


  //----------------VIRTUINO BLUETOOTH------------------
  /*========= Virtuino General methods
     void vDigitalMemoryWrite(int digitalMemoryIndex, int value)   write a value to a Virtuino digital memory   (digitalMemoryIndex=0..31, value range = 0 or 1)
     int  vDigitalMemoryRead(int digitalMemoryIndex)               read  the value of a Virtuino digital memory (digitalMemoryIndex=0..31, returned value range = 0 or 1)
     void vMemoryWrite(int analogMemoryIndex, float value)         write a value to Virtuino float memory       (memoryIndex=0..31, value range as float value)
     float vMemoryRead(int analogMemoryIndex)                      read the value of  Virtuino analog memory    (analogMemoryIndex=0..31, returned value range = 0..1023)
     run()                                                         neccesary command to communicate with Virtuino android app  (on start of void loop)
     void vDelay(long milliseconds);                               Pauses the program (without block communication) for the amount of time (in miliseconds) specified as parameter
     int getPinValue(int pin)                                      read the value of a Pin. Usefull for PWM pins
  */
  virtuino.run();


  //---------------bOUTON----------------------
  BPVal = analogRead(BP);

  //-----Exemple de Tempo MESURE VITESSE-----------------
  //-----T[1]
  Cond[1] = !TDN[1];
  TDN[1] = Tempos( 1, 10);
  if (TDN[1]) {
    Vitaux1 = (((3.1416 * 42) / 360) * compteur1) / 10;
    compteur1 = 0;
    Vitaux2 = (((3.1416 * 42) / 360) * compteur2) / 10;
    compteur2 = 0;
  }
  Vit1 = ((Vit1 * 10) + Vitaux1) / (10 + 1);
  Vit2 = ((Vit2 * 10) + Vitaux2) / (10 + 1);

  //-----------Gyro-----------------------
  accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.getRotation(&gx, &gy, &gz);



  //-------------BOUSOLE--------------
  int x, y, z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();


  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //MSB  x
    x |= Wire.read(); //LSB  x
    z = Wire.read() << 8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read() << 8; //MSB y
    y |= Wire.read(); //LSB y
  }

  //---------------ESSAIMOTEUR---------------

  if (BPVal == 0) {
    analogWrite(M1CW, 255);
    digitalWrite(M1CCW, LOW);
    analogWrite(M2CW, 255);
    digitalWrite(M2CCW, LOW);
  } else {
    digitalWrite(M1CCW, HIGH);
    digitalWrite(M1CW, HIGH);
    digitalWrite(M2CCW, HIGH);
    digitalWrite(M2CW, HIGH);
  }

  //---------------ENCODEUR---------------
  // M1CCWENCIN = digitalRead(M1CCWENCIN);

  //--1er Filtre
  //  for (int i = 0; i < MeasurementsToAverage; ++i)
  //  {
  //    AverageMesure += analogRead(M1CWENC);
  //    delay(1);
  //  }
  //  AverageMesure /= MeasurementsToAverage;
  // M1CWENCIN = AverageMesure;


  //--2em Filtre
  //  float RawTemperature = analogRead(M1CWENC);
  //
  //  RunningAverageBuffer[NextRunningAverage++] = RawTemperature;
  //  if (NextRunningAverage >= RunningAverageCount)
  //  {
  //    NextRunningAverage = 0;
  //  }
  //  float RunningAverageTemperature = 0;
  //  for(int i=0; i< RunningAverageCount; ++i)
  //  {
  //    RunningAverageTemperature += RunningAverageBuffer[i];
  //  }
  //  RunningAverageTemperature /= RunningAverageCount;
  //
  //  delay(100);
  //M1CWENCIN=RunningAverageTemperature;


  //--3em Filtre
  //  float RawTemperature = analogRead(M1CWENC);
  //  FilteredTemperature.Filter(RawTemperature);
  //  float SmoothTemperature = FilteredTemperature.Current();
  //  M1CWENCIN = SmoothTemperature;


  //--4em Filtre
  //M1CWENCIN = ((M1CWENCIN * 50) + analogRead(M1CWENC)) / (50 + 1);
  // M1CWENCIN = digitalRead(M1CWENC);
  //M1CCWENCIN = digitalRead(M1CCWENC);


  //-------------------COMMUNICATION VIRTUINO--------------------
  virtuino.vMemoryWrite(0, Vit1);

  //------------------------MESURE VITESSE-------------------



  //------------megu1olink
  // if ((millis() - LastSent) > SendInterval)
  //{
  //LastSent = millis();
  //    MyPlot.SendData("ax", ax);
  //    MyPlot.SendData("ay", ay);
  //    MyPlot.SendData("az", az);

  //    MyPlot.SendData("ax", x);
  //    MyPlot.SendData("ay", y);
  //    MyPlot.SendData("az", z);
  //
  //    MyPlot.SendData("BPVal", BPVal);
  // MyPlot.SendData("M1CWENCIN", M1CWENCIN);
  //MyPlot.SendData("M1CCWENCIN", M1CCWENCIN);
  MyPlot.SendData("Vit1", Vit1);
  MyPlot.SendData("Vit2", Vit2);
  // }
}

//***************************FIN LOOP******************************************

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
