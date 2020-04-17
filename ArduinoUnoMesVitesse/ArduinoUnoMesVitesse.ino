#include <Arduino.h>
#include "MegunoLink.h"

//----Mesure de vitesse cpt impulsions
//----Mesure de vitesse cpt impulsions
#define enc1 2
#define enc2 3

float compteur1 = 0;
float compteur2 = 0;

boolean pulseSign = false;

//----- MEGUOLINK-------------
// Millis value when the data was last sent.
long LastSent;
// Interval (milliseconds) between sending analog data
const unsigned SendInterval = 1; // [ms]
// The plot we are sending data to. A TimePlot is used here
long LastSent2;
const unsigned SendInterval2 = 200; // [ms]
TimePlot MyPlot; //no channel selected



//---------------------VOID IRQ CODEUR MESURE DE VITESSE----
void isr1() {
  noInterrupts();
  compteur1 = compteur1 + 1;
  pulseSign = !pulseSign;
  interrupts();
}
void isr2() {
  noInterrupts();
  compteur2 = compteur2 + 1;
  delay(10);
  interrupts();
}

//*********************************SETUP***********************
void setup() {

  Serial.begin(115200);

  //-----------MESURE VITESSE-------ATTENTION TROP RAPIDE------A FAIRE FAIRE PAR UN AUTRE CPU---
  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1), isr1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(enc2), isr2, CHANGE );

  LastSent = millis();
  MyPlot.SetTitle("My Analog Measurement");
  MyPlot.SetXLabel("Time");
  MyPlot.SetYLabel("Value");
  MyPlot.SetSeriesProperties("IRQ", Plot::Magenta, Plot::Solid, 1, Plot::NoMarker);

}

//********************LOOP*********************************
void loop() {

  //Serial.print("Compteur1:");
  // Serial.println(pulseSign);
  // Serial.print("Compteur2:");
  // Serial.println(compteur2);


  //------------meguolink

  if ((millis() - LastSent) > SendInterval)
  {
    LastSent = millis();
    MyPlot.SendData("compteur1", compteur1);
    MyPlot.SendData("IN Change", pulseSign);
  }
  if ((millis() - LastSent2) > SendInterval2)
  {
    LastSent2 = millis();
    compteur1 = 0;
  }


  delay(10);
}
