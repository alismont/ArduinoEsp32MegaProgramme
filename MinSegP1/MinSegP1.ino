/*
  Alain Lismont

*/

#include <Arduino.h>
#include "MegunoLink.h"
#include "Wire.h"
#include "TimerOne.h"

#include <PID_v1.h>


//---------------ONS------------------
int B3[10];

//-----------PID VITESSE--------------
//Define Variables we'll be connecting to
double Setpoint, Input, Output, CSG_OUTPUTV1;

//Specify the links and initial tuning parameters
double Kp = 10, Ki = 60, Kd = 1.4;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//-----------PID POS--------------
//Define Variables we'll be connecting to
double SetpointPos, InputPos, OutputPos, CSG_OUTPUTV1Pos;

//Specify the links and initial tuning parameters
double KpPos = 4, KiPos = 0, KdPos = 0;
PID myPIDPos(&InputPos, &OutputPos, &SetpointPos, KpPos, KiPos, KdPos, DIRECT);


//--------VARIABLES POT----------------
int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor


//---------VARIABLES GIRO--------------
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;
double Angle = 0;
double AngleMemo = 0;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high



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
int BPValMemo = 0;

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


//----VARIABLES Me0sure de vitesse cpt impulsions----------------------------------
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
double Vit1 = 0;

double Vitaux2 = 0;
double Vit2 = 0;
volatile float compteur1 = 0;
volatile float compteur2 = 0;

double RefPos = 115;

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

}

//----------------BLUETOOTH---------------
#include "VirtuinoBluetooth.h"                           // Include VirtuinoBluetooth library to your code
//#include <SoftwareSerial.h>                              //  Disable this line if you want to use hardware serial
//SoftwareSerial bluetoothSerial =  SoftwareSerial(2,3);   // arduino RX pin=2  arduino TX pin=3    connect the arduino RX pin to bluetooth module TX pin   -  connect the arduino TX pin to bluetooth module RX pin.  Disable this line if you want to use hardware serial
VirtuinoBluetooth virtuino(Serial2, 9600);

//--------------GESTION IRQ GIRO--------------
void dmpDataReady()
{
  mpuInterrupt = true;
}


//******************SETUP*****************************
void setup() {

  Serial.begin(115200);

  //------------------PID POSITION--------------
  myPIDPos.SetMode(AUTOMATIC);
  myPIDPos.SetOutputLimits(-255, 255);
  myPIDPos.SetSampleTime(10);

  //------------------PID VITESSE--------------
  //  myPID.SetMode(AUTOMATIC);
  //  myPID.SetOutputLimits(0, 255);
  //  myPID.SetSampleTime(10);

  //---------VIRTUINO BLUTOOTH------------------
  virtuino.DEBUG = false;


  //------------GYRO---------------------
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(3, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();



  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

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
  // attachInterrupt(digitalPinToInterrupt(18), INTer1, HIGH );
  // attachInterrupt(digitalPinToInterrupt(19), INTer2, HIGH );


  //-----------MEGUOLINK
  //LastSent = millis();

  MyPlot.SetTitle("My Analog Measurement");
  MyPlot.SetXLabel("Time");
  MyPlot.SetYLabel("Value");

  //MyPlot.SetSeriesProperties("SetpointPos", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);
  //MyPlot.SetSeriesProperties("angle_roll_output", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  // MyPlot.SetSeriesProperties("InputPos", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  //MyPlot.SetSeriesProperties("CSG_OUTPUTV1Pos", Plot::Black, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("x", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("y", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  //  MyPlot.SetSeriesProperties("z", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  //
  //MyPlot.SetSeriesProperties("BPVal", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  //MyPlot.SetSeriesProperties("BPValMemo", Plot::Red, Plot::Solid, 1, Plot::NoMarker);

  // MyPlot.SetSeriesProperties("M1CWENCIN", Plot::Green, Plot::Solid, 1, Plot::NoMarker);

  // MyPlot.SetSeriesProperties("OutputPos", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("OutputPos", Plot::Green, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("SetpointPos", Plot::Red, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("Angle", Plot::Blue, Plot::Solid, 1, Plot::NoMarker);

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



  //----------------------LECTURE GYRO-----------------
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors
    Serial.println("foutu");
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if LOG_INPUT
    // Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180/M_PI);
    // Serial.print("\t");
    //Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\t");
    //Serial.println(ypr[2] * 180/M_PI);
#endif
    Angle = ypr[2] * 180 / M_PI + 180;
    AngleMemo = ((AngleMemo * 6) + Angle) / 7;
    // Serial.print ("Angle: ");
    //Serial.println(Angle);
  }


  //---------------BOUTON--ONS--------------------
  BPVal = analogRead(BP);
  if (BPVal == 0) { //-cond ons
    if (!B3[1]) {  //---bit ons
      B3[1] = 1.0;
      B3[2] = 1.0;  //----Action
    }
    else {
      B3[2] = 0.0; //------erase action au scan suivant
    }
  }
  else {
    B3[1] = 0.0;//---plus la condition
    B3[2] = 0.0;
  }

  //--ACTION

  if ((B3[2] == 1 ) & (BPValMemo == 0)) {
    BPValMemo = 1;
  } else {
    if ((B3[2] == 1 ) & (BPValMemo == 1)) {
      BPValMemo = 0;
    }
  }

  //-----Exemple de Tempo MESURE VITESSE-----------------
  //-----T[1]
  Cond[1] = !TDN[1];
  TDN[1] = Tempos( 1, 10);
  if (TDN[1]) {
    Vitaux1 = 0.37 * compteur1;
    compteur1 = 0;
    Vitaux2 = 0.37 * compteur2;
    compteur2 = 0;
  }
  Vit1 = ((Vit1 * 10) + Vitaux1) / (10 + 1);
  Vit2 = ((Vit2 * 10) + Vitaux2) / (10 + 1);



  //-----------------PID POSITION DANS ROUTINE GYRO-------------
  SetpointPos = 185; //116.0; map(analogRead(sensorPin), 0 , 800, 0, -100);
  InputPos = Angle;

  myPIDPos.Compute();
  CSG_OUTPUTV1Pos = abs( OutputPos);
  if ((CSG_OUTPUTV1Pos > 1) & (CSG_OUTPUTV1Pos < 100) ) CSG_OUTPUTV1Pos = 100;
  //-----------------PID VITESSE-------------
  // Setpoint = CSG_OUTPUTV1Pos;
  // Input = Vit1;

  //myPID.Compute();
  //CSG_OUTPUTV1 = Output;


  //---------------ESSAIMOTEUR---------------

  if (BPValMemo == 1) {
    if (OutputPos >= 0) {
      analogWrite(M1CW, CSG_OUTPUTV1Pos);
      digitalWrite(M1CCW, LOW);
      analogWrite(M2CW, CSG_OUTPUTV1Pos);
      digitalWrite(M2CCW, LOW);
    } else {
      analogWrite(M1CCW, CSG_OUTPUTV1Pos);
      digitalWrite(M1CW, LOW);
      analogWrite(M2CCW, CSG_OUTPUTV1Pos);
      digitalWrite(M2CW, LOW);
    }
  } else {
    digitalWrite(M1CCW, HIGH);
    digitalWrite(M1CW, HIGH);
    digitalWrite(M2CCW, HIGH);
    digitalWrite(M2CW, HIGH);
    //MiseAiP();
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
  RefPos = virtuino.vMemoryRead(1);

  //    KpPos = virtuino.vMemoryRead(2);
  //    KiPos = virtuino.vMemoryRead(3);
  //    KdPos = virtuino.vMemoryRead(4);
  //
  //  Kp = virtuino.vMemoryRead(5);
  //  Ki = virtuino.vMemoryRead(6);
  //  Kd = virtuino.vMemoryRead(7);

  virtuino.vMemoryWrite(10, InputPos);
  virtuino.vMemoryWrite(11, SetpointPos);
  //------------------------MESURE VITESSE-------------------



  //------------MEGUNOLINK
  // if ((millis() - LastSent) > SendInterval)
  //{
  //LastSent = millis();

  //MyPlot.SendData("SetpointPos", SetpointPos);
  // MyPlot.SendData("angle_roll_output", angle_roll_output);
  //MyPlot.SendData("InputPos", InputPos);
  //MyPlot.SendData("CSG_OUTPUTV1Pos", CSG_OUTPUTV1Pos);

  //    MyPlot.SendData("x", x);
  //   MyPlot.SendData("y", y);
  //    MyPlot.SendData("z", z);
  //
  //  MyPlot.SendData("BPVal", BPVal);
  //MyPlot.SendData("BPValMemo", BPValMemo);
  //MyPlot.SendData("OutputPos", OutputPos);
  MyPlot.SendData("OutputPos", OutputPos);
  MyPlot.SendData("SetpointPos", SetpointPos);
  MyPlot.SendData("Angle", Angle);
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

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
}

void MiseAiP() {
  //------------------PID POSITION--------------
  myPIDPos.SetMode(AUTOMATIC);
  myPIDPos.SetOutputLimits(-290, 290);
  myPIDPos.SetSampleTime(10);

  //------------------PID VITESSE--------------
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(10);
}
