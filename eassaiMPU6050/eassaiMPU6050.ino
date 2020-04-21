/*
 * based on the GyRobo Code "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * No balancing 
 *  * Gyro MPU6050 on the NoRobo but it's used only to get data, not to change anything
 * 
 * This is a mix of NoRobo-Joystick-BT-commander-2016-04-07.ino & the balancing GyRobo code
 * Still not balancing - just reading angle and acceleration and calculating angle
 * 
 * Reste 1 défaut : au démarrage, le moteur D fait des à coups
 * 
 * HC-05 Bluetooth setup for 57 600 baud on the TX and RX pins
 * Joystick BT Commander Version 5.2 
 * Button #1: "ON", Button #2: "Fast"
 * 
 * Data field #1: "data1", Data field #2: "Power", Data field #3: "Cycle (ms)" 
 * note: cycle time must not exceed 4ms
 */


#pragma GCC optimize ("3")


//main loop--------------------------------------------------------
#define LEDPIN   13

bool power = false;            //run only when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running

//angle calculations from MPU-6050--------------------------------------------------
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
float robot_angle;
float Acc_angle;            //angle calculated from acc. measurments

bool vertical = false;      //is the robot vertical enough to run

bool GyY_filter_on = true;  //apply simple average filter to Z gyro reading

#define Gyro_amount 0.996   //percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s





/****************************************************************************************************
 * SETUP
 ****************************************************************************************************/
 
void setup()
{
  pinMode (LEDPIN, OUTPUT);
 
  // Start Serial Port
  Serial.begin(57600);
  Serial.println("NoRobo_with_bluetooth_working_V0.1" );
  Serial.println("The NoRobo should be on its head, (wheels up !)" );
  Serial.println("place robot so that MPU6050 is horizontal withing 5sec. of power for gyroscope calibration");

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //setup robot angle calculation
  Serial.println("calibrating gyroscope...........");
  angle_init();  
  Serial.println("GYRO Setup is DONE"); 

  Serial.println("place robot vertical to run"); 

  // empty RX buffer
  while (Serial.available())  Serial.read();        
}

/****************************************************************************************************
 * LOOP
 ****************************************************************************************************/
 
void loop()
{
  //run main loop every ~4ms
  if ((freqCounter & 0x07f) == 0)
  {
    // record when loop starts
    oldfreqCounter = freqCounter;
    //calculate angle of robot
    angle_calc();

    /***********************
     * to print the values in a table
    ***********************/
    
      Serial.print("GyY = "); Serial.print(GyY); Serial.print("\t");
      Serial.print(" | AcX = "); Serial.print(AcX); Serial.print("\t");
      Serial.print(" | AcZ = "); Serial.print(AcZ); Serial.print("\t");
      Serial.print(" | robot_angle = "); Serial.print(robot_angle); Serial.print("\t"); 
      Serial.print(" | Acc_angle = "); Serial.print(Acc_angle); Serial.print("\t");
      // Serial.print(" | vertical = "); Serial.print(vertical); Serial.print("\t");
      Serial.print("\n");
      delay(1000);
    /************************/


    //calculate loop time
    if (freqCounter > oldfreqCounter)
    {
      if (loop_time < (freqCounter - oldfreqCounter)) loop_time = freqCounter - oldfreqCounter;
    }

  }
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
void angle_init()
{
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
      Serial.println(GyY);
      GyY_offset_sum += GyY;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
    GyY_filter_on = true;
  }
  else
  {
    for (int i = 0; i < 1024; i++)
    {
      angle_calc();
      Serial.println(GyY);
      GyY_offset_sum += GyY;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = ");
  Serial.println(GyY_offset);
}


//calculate robot tilt angle
void angle_calc()
{
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050,2,true);  // request a total of 2 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);          // starting with register 0x3F (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);       // starting with register 0x45 (GYRO_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

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
  Acc_angle =  atan2(AcX, -AcZ) * 57.2958;              //angle from acc. values       * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);

  
  //check if robot is vertical
  if (robot_angle > 50 || robot_angle < -50) vertical = false;
  if (robot_angle < 1 && robot_angle > -1) vertical = true;


 
}




//--------------------------------------------------------------
// code loop timing---------------------------------------------
//--------------------------------------------------------------
// minimize interrupt code length
// is called every 31.875us (510 clock cycles)  ???????
ISR( TIMER1_OVF_vect )
{
  //every 32 count of freqCounter ~1ms
  freqCounter++;

  if ((freqCounter & 0x01) == 0)
  {
    /*
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
    */
  }
}
