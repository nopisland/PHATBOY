
/*

 **************************************
                PHATBOY
 **************************************

  Mini-sumo robot "Phatboy"
  for designing and building projects
  at GEII dpt in University of Bordeaux.


  -------------------------------------
  Version name : V3
  Last modified on : 20/12/2016 20:31
  -------------------------------------

  Changelog :

    Version 1 :
   -----------

  - Added battery handling
  - Added distance sensors calibration
  - Added line sensors calibration
  - Added full power at push

  - Deleted unused code

  - Fixed colors
  - Fixed random rotation at start
  - Fixed weird servo moves

    Version 2 :
   -----------

  - Added direction choise

   Version 3 :
   -----------

  - Added detection while turning
  - Added smart line calibration

  - Inproved line detection issues

  Version 4 :
  -----------

  - Modified mecanical part include these modification in the code:
    - Angle modified for the servo
    - Two different type of servo for two différent vertion of robot (please select robot before flash)
    - Adding debug mode


*/
// You nust to choose one robot to flash because there is 2 différents type of servo inside and the controls are not the same
#define PhatboyBlue
//#define PhatboyViolet
//#define DEBUG

#include <Wire.h>
#include <Servo.h>
#include "VL53L0XMod.h"

#include "PHATBOY.Constantes.h"
#include "PHATBOY.Fonctions.h"



//IR



VL53L0X sensor;
VL53L0X sensor1;

Servo servo;

unsigned long timeSinceStartWaiting, timeBack;
bool lLine, rLine, bLine;
int test_trame = 1;
int etat = 0;

int mesuredBlack = 0;
int offset = 0;
int blackDiff = 100;

unsigned long timeFocus = 0;

//bool goToTheLeft = HIGH;
bool goToTheLeft = HIGH;

bool avoiding;

void Tuple(unsigned long dist, unsigned long dist1)
{
  Serial.print(dist1);
  Serial.print(",");
  Serial.println(dist);
}

void setup () {
  InitPins();
  pinMode(remote, INPUT_PULLUP);

  servo.attach(OUT_SERVO);
  SetArm(LOW);


  digitalWrite(LED_B, LOW);


  Serial.begin(115200);

  digitalWrite(OUT_NMOS, HIGH);




  delay(250);
  InitSensors();
  Serial.print("first print");
  Serial.println(sensor1.getAddress(), HEX);
  Calibrate();
  servo.detach();
}

void loop () {
#ifdef DEBUG
  Serial.println("DEBUG");
  //Debug_Aquisition();
  Debug_Action();



#else
//Serial.println(etat);
  static uint32_t tdetach = 0;
  static byte need_detach = false;


  switch (etat) {
    case 0:// test de la trame afin de voir qu'il ne s'agit pas de perturbations
    
             if (digitalRead(remote) == LOW) {

    unsigned long debutON = micros();

    // Mesurer la duree du signal ON
    while (digitalRead(remote) == LOW) {
      if (micros() - debutON > HEADER_ON_MAX) {
        return;
      }
    }

    unsigned long dureeON = micros() - debutON;

    // Verifier la duree du ON (environ 9 ms)
    if (dureeON >= HEADER_ON_MIN &&
        dureeON <= HEADER_ON_MAX) {

      unsigned long debutOFF = micros();

      // Mesurer la duree du OFF
      while (digitalRead(remote) == HIGH) {
        if (micros() - debutOFF > HEADER_OFF_MAX) {
          return;
        }
      }

      unsigned long dureeOFF = micros() - debutOFF;

      // Verifier la duree du OFF (environ 4,5 ms)
      if (dureeOFF >= HEADER_OFF_MIN &&
          dureeOFF <= HEADER_OFF_MAX) {

        Serial.println("Debut d'une trame NEC detecte !");
        
        // La conception s'arrete ici
        etat =1;
      }
        }
             }
  


      break;

    case 1:
Serial.println("etat1");



      digitalWrite(LED_B, HIGH);
      timeSinceStartWaiting = millis();
      servo.attach(OUT_SERVO);
      Calibrate();
      etat = 2;

      break;

    case 2 :
      etat = 3; // bypass 5s waiting time
      if ((millis () - timeSinceStartWaiting) % 1000 > 500) {

      } else {
        if (goToTheLeft) {

        } else {

        }
      }

      if (GetStartButton () && (millis () - timeSinceStartWaiting) > 200) {
        goToTheLeft = !goToTheLeft;
      }

      if (timeSinceStartWaiting + WAITTIME < millis ()) {
        etat = 3;
      }
      break;

    case 3 :
      SetArm (HIGH);

      etat = 4;
      SetSpeed (255, 255);
      need_detach = true;
      tdetach = millis();
      break;

    case 4 :
      if (need_detach && millis() - tdetach > 500)
      {
        need_detach = false;
        servo.detach ();
      }

      if (goToTheLeft) {
        SetSpeed (-255, 255);
      } else {
        SetSpeed (255, -255);
      }
      delay (250 * 6 / GetBattery ());
      etat = 5;
      break;

    case 5 :

      GetLines ();
      AvoidLines ();

      if (!avoiding) {
        FocusRobot ();
        //SetSpeed (255, 255);
      }

      if (!lLine && !rLine && !bLine) {

      } else {

      }

      /*if (GetStartButton ()) { // disable as push button is replaced by TSOP then avoid faulty reset*/
      /*
              Reset ();
            }

            break;

        }

        bool batteryIsLow = IsBatteryLow (GetBattery ());

        if (batteryIsLow) {
          if (millis () % 1000 > 500) {

          } else {

          }
        } else {

        }
      */

  }

#endif
} 
