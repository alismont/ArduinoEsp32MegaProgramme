
/* Processing code for this example
 
 // Graphing sketch
 
 
 // Ce programme re�oit des chaine de caract�re ASCII
 // depuis le port S�rie � 115200 bauds et les affiche sous forme de courbe
 // dans une fourchette de 0 to 1023, 
 //lorsque les donn�es sont suivies d'un retour de ligne
 
 // Created 20 Apr 2005
 // Updated 18 Jan 2008
 // by Tom Igoe 
 //
 // modifi� par X. HINAULT - January 2010
 // www.mon-club-elec.fr
 */

import processing.serial.*;

Serial myPort;        // Variable Port S�rie
int xPos = 1;         // variable abscisse  - x
int xPos0=1;       // variable m�morisation xPos n-1

float yPos=1;  // variable yPos - ordonn�e
float yPos0=1; // variable yPos n-1

void setup () {

  // initialise la fen�tre 
  size(1000, 1000);        
  // 90% de l'�cran par d�faut - r�gler au besoin - 
  // viser taille maxi en hauteur - taille fen�tre (x,y)


  // Liste tous les ports disponible et affiche le r�sultat 
  println(Serial.list());

  // Le port COM3 est list� avec l'indice 1
  // donc j'ouvre le port Serial.list()[1].
  // A adapter si votre port est diff�rent - cf liste qui s'affiche � l'ex�cution
  myPort = new Serial(this, Serial.list()[0], 115200);
  // ne g�n�re aucun �v�nement S�rie tant qu'aucun caract�re saut de ligne n'est re�u
  myPort.bufferUntil('\n');
  // initialise le fond de la fen�tre
  background(255);// 0 = noir - 255 = blanc
}
void draw () {
  // tout se passe dans la fonction SerialEvent car le programme re�oit des donn�es
  while (myPort.available() > 0) {
    String inString = myPort.readStringUntil('\n');

    if (inString != null) {
      // enl�ve les espaces
      inString = trim(inString);
      // convertit la cha�ne en valeur num�rique 
      float inByte = float(inString); 
      // r�-�chelonne la valeur pour affichage
      inByte = map(inByte, 0, 1000, 0, height);

      yPos=inByte; // l'ordonn�e est la valeur re�ue par le port s�rie

      // trace la ligne
      stroke(0, 0, 255); // fixe la couleur utilis�e pour le trac� en RVB 

      line (xPos0, height-yPos0, xPos, height-yPos); // trace une ligne en tenant compte valeur re�ue

      xPos0=xPos; // m�morisation xPos n-1
      yPos0=yPos; // m�morisation xPos n-1

      // � la fin de l'�cran revient au d�but
      if (xPos >= width) {
        xPos = 0;
        xPos0=0; // pour retour de la trace sans ligne

        background(255); // 0 pour noir - 255 pour blanc...
      } else {
        // incr�mente la position horizontale (abscisse)
        xPos++;
      }
    }
  }
}