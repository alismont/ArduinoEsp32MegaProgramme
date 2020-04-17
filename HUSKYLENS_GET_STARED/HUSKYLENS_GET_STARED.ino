/***************************************************
  HUSKYLENS An Easy-to-use AI Machine Vision Sensor
  <https://www.dfrobot.com/product-1922.html>

 ***************************************************
  This example shows the basic function of library for HUSKYLENS via Serial.

  Created 2020-03-13
  By [Angelo qiao](Angelo.qiao@dfrobot.com)

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
  1.Connection and Diagram can be found here
  <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
  2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "MegunoLink.h"
#include "Wire.h"

// l’adresse indiquée est dans mon cas  "0x27"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20 for a 16 chars and 2 line display

int count = 0;
HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);
int XPosCentre = 0;


//----- MEGUOLINK-------------
// Millis value when the data was last sent.
long LastSent;
// Interval (milliseconds) between sending analog data
const unsigned SendInterval = 200; // [ms]
// The plot we are sending data to. A TimePlot is used here
TimePlot MyPlot;

//*************SETUP**********************

void setup() {
  Serial.begin(9600);
 // lcd.init();                      // initialize the lcd 


//--------------WIRE-------------
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event

 
  // Print a message to the LCD.
 // lcd.backlight();
//  lcd.print("Hello, world!");

  //-----------MEGUOLINK
  LastSent = millis();

//  MyPlot.SetTitle("My Analog Measurement");
//  MyPlot.SetXLabel("Time");
//  MyPlot.SetYLabel("Value");
//  MyPlot.SetSeriesProperties("ADCValue", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);

  mySerial.begin(9600);
  while (!huskylens.begin(mySerial))
  {
//    Serial.println(F("Begin failed!"));
//    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
//    Serial.println(F("2.Please recheck the connection."));
//    delay(100);
  }


//  for (byte i = 8; i < 120; i++)
//  {
//    Wire.beginTransmission (i);
//    if (Wire.endTransmission () == 0)
//    {
//      Serial.print ("L'adresse trouvé: ");
//      Serial.print (i, DEC);
//      Serial.print (" (0x");
//      Serial.print (i, HEX);
//      Serial.println (")");
//      count++;
//      delay (1);
//    }
//
//  }
delay(10000);
}
//**********************LOOP************************
void loop() {
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else
  {
    Serial.println(F("###########"));
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
    }
  }

  //------------megu1olink

  if ((millis() - LastSent) > SendInterval)
  {
    LastSent = millis();
   // MyPlot.SendData("Vitesse", XPosCentre);
  }

  //*************I2C*****************
//   Wire.beginTransmission(39); // transmit to device #1
//    Wire.write("XPosCentre"); // sends
//    Wire.endTransmission(); // stop transmitting



}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    XPosCentre = result.xCenter;
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  }
  else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  }
  else {
    Serial.println("Object unknown!");
  }
}

//------------------------WIRE------------------
void requestEvent() {
  int c=XPosCentre;
  Wire.write(c); // respond with message of 6 bytes
  // as expected by master
}
