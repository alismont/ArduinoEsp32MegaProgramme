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
double Kp = 0, Ki = 60, Kd = 100;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);


//-----------PID POS--------------
//Define Variables we'll be connecting to
double SetpointPos, InputPos, OutputPos, CSG_OUTPUTV1Pos;

//Specify the links and initial tuning parameters
double KpPos = 1.9, KiPos = 0.20, KdPos = 0.12;
double KpPosD = 0.5, KiPosD = 0.20, KdPosD = 0.12;
PID myPIDPos(&InputPos, &OutputPos, &SetpointPos, KpPos, KiPos, KdPos, DIRECT);


//--------VARIABLES POT----------------
int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor


//---------VARIABLES GIRO--------------
#include <Wire.h>

#define MPU6050 0x68              //Device address (standard)
#define ACCEL_CONFIG 0x1C         //Accelerometer configuration address
#define GYRO_CONFIG 0x1B          //Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C


//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


int16_t  AcZ, AcX, GyY;       // my gyro with x forward, y left-right, z vertical

int16_t  AcZ_offset = 0;
int16_t  AcX_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyY_offset_sum = 0;

int32_t GyY_filter[32];
uint8_t filter_count = 0;
int32_t  GyY_F;
double robot_angle;

;
double Acc_angle, Acc_angleMemo, DiffPos;          //angle calculated from acc. measurments

bool vertical = false;      //is the robot vertical enough to run

bool GyY_filter_on = true;  //apply simple average filter to Z gyro reading

#define Gyro_amount 0.996   //percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s



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



//******************SETUP*****************************
void setup() {


  Serial.begin(115200);


  //------------------PID POSITION--------------
  myPIDPos.SetMode(AUTOMATIC);
  myPIDPos.SetOutputLimits(-255, 255);
  myPIDPos.SetTunings(KpPos, KiPos, KdPos);
  myPIDPos.SetSampleTime(10);

  //------------------PID VITESSE--------------
  //  myPID.SetMode(AUTOMATIC);
  //  myPID.SetOutputLimits(0, 255);
  //  myPID.SetSampleTime(10);

  //---------VIRTUINO BLUTOOTH------------------
  virtuino.DEBUG = false;


  //------------GYRO---------------------
  angle_init();


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
  MyPlot.SetSeriesProperties("Acc_angle", Plot::Blue, Plot::Solid, 1, Plot::NoMarker);

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
  angle_calc();
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
  SetpointPos = 78; //116.0; map(analogRead(sensorPin), 0 , 800, 0, -100);
  InputPos = Acc_angle;
  //Bande Morte
  DiffPos = abs(SetpointPos - InputPos);
  if  (DiffPos < 1) {
    myPIDPos.SetTunings(KpPosD, KiPosD, KdPosD);
  } else {
    myPIDPos.SetTunings(KpPos, KiPos, KdPos);
  }

  myPIDPos.Compute();
  CSG_OUTPUTV1Pos = abs( OutputPos) + 50;
  //if ((CSG_OUTPUTV1Pos > 1) & (CSG_OUTPUTV1Pos < 100) ) CSG_OUTPUTV1Pos = 100;


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
  MyPlot.SendData("Acc_angle", Acc_angle);
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

//------------------------------------------------------------------
//Robot angle calculations------------------------------------------
//------------------------------------------------------------------

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

//setup MPU6050
void angle_init() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  // calc Z gyro offset by averaging 1024 values
  if (GyY_filter_on == true)
  {
    GyY_filter_on = false;
    for (int i = 0; i < 1024; i++)
    {
      angle_calc();
      //    //Serial.println(GyY);
      GyY_offset_sum += GyY;
      //      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
    GyY_filter_on = true;
  }
  else
  {
    for (int i = 0; i < 1024; i++)
    {
      angle_calc();
      ////Serial.println(GyY);
      GyY_offset_sum += GyY;
      //      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
  }
  GyY_offset = GyY_offset_sum >> 10;
  // Serial.print("GyY offset value = ");
  //Serial.println(GyY_offset);
}


//calculate robot tilt angle
void angle_calc()
{
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true); // request a total of 2 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);          // starting with register 0x3F (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);       // starting with register 0x45 (GYRO_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

  if (GyY_filter_on == true)
  {
    // simple low pass filter on gyro
    GyY_filter[filter_count] = GyY;

    filter_count++;

    if (filter_count > 15) filter_count = 0;

    GyY_F = 0;
    for (int i = 0; i < 16; i++)
    {
      GyY_F += GyY_filter[i];
      GyY = GyY_F >> 4;
    }
  }

  // add mpu6050 offset values
  AcZ += AcZ_offset;
  AcX += AcX_offset;
  GyY += GyY_offset;

  //use complementary filter to calculate robot angle
  robot_angle -= GyY * 6.07968E-5;                      //integrate gyroscope to get angle       * 0.003984 (sec) / 65.536 (bits / (deg/sec))
  //robot_angle += GyY * 6.07968E-5;                      //integrate gyroscope to get angle       * 0.003984 (sec) / 65.536 (bits / (deg/sec))
  Acc_angleMemo =  atan2(AcX, -AcZ) * 57.2958;              //angle from acc. values       * 57.2958 (deg/rad)
  Acc_angle = (((Acc_angle * 4) + Acc_angleMemo) / 5);
  robot_angle = robot_angle * Gyro_amount + Acc_angleMemo * (1.0 - Gyro_amount);


  //check if robot is vertical
  if (robot_angle > 50 || robot_angle < -50) vertical = false;
  if (robot_angle < 1 && robot_angle > -1) vertical = true;
}
