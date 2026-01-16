
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

*/


#include <Wire.h>
#include <Servo.h>
#include "VL53L0XMod.h"

#include "PHATBOY.Constantes.h"
#include "PHATBOY.Fonctions.h"

VL53L0X sensor;
VL53L0X sensor1;

Servo servo;

uint32_t timeSinceStartWaiting, timeBack;
bool lLine, rLine, bLine;
bool avoiding_left = LOW, avoiding_right = LOW, avoiding_back = LOW;

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
  InitPins ();
  
  
  servo.attach (OUT_SERVO);

  Serial.begin (115200);

  digitalWrite (OUT_NMOS, HIGH);

  SetArm (LOW);
  SetLED (HIGH) ;
  SetRGB (255, 255, 0);

  delay(100);
  InitSensors ();

  delay(500);
  Serial.println("coucou");
Calibrate ();
Serial.print("LEFT  : "); Serial.println(analogRead (IN_IRLEFT));
Serial.print("RIGHT : "); Serial.println(analogRead (IN_IRRIGHT));
Serial.print("BACK  : "); Serial.println(analogRead (IN_IRBACK));

  servo.detach ();
}

void loop () {
    static byte iteration_ct = 0;
    static uint32_t tdetach = 0;
    static byte need_detach = false;
    static uint32_t last_focus = 0;

  switch (etat) {
    case 0 :

      // Waiting until the button is pushed

      if (GetStartButton ()) {
        SetRGB (255, 255, 255); // WHITE
        timeSinceStartWaiting = millis();
        Calibrate ();
        servo.attach (OUT_SERVO);
        etat = 1;
      }
      break;

    case 1 :
      etat = 2; // bypass 5s waiting time
      if ((millis () - timeSinceStartWaiting) % 1000 > 500) {
        SetRGB (0, 0, 0); // BLACK
      } else {
        if (goToTheLeft) {
          SetRGB (255, 255, 255); // WHITE
        } else {
          SetRGB (255, 0, 170); // PINK
        }
      }

      if (GetStartButton () && (millis () - timeSinceStartWaiting) > 200) {
        goToTheLeft = !goToTheLeft;
      }

      if (timeSinceStartWaiting + WAITTIME < millis ()) {
        etat = 2;
      }
      break;

    case 2 :
      SetArm (HIGH);
      SetRGB (0, 0, 255); // RED
      etat = 3;
      SetSpeed (255, 255);
      need_detach = true;
      tdetach = millis();
      break;

    case 3 :
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
      etat = 4;
      break;

    case 4 :
      
      GetLines ();
      AvoidLines ();

       if (avoiding)
            SetRGB (255, 255, 255); //MAGENTA
       else
            SetRGB (0, 0, 0); //MAGENTA

      if (!avoiding && millis() - last_focus > 50) {
        last_focus = millis();
        SetRGB (0, 255, 0);
      //  FocusRobot ();
        
      }
      
      if (!lLine && !rLine && !bLine) {
        
      } else {
      //  SetRGB (255, 0, 255); //MAGENTA      
      }

      /*if (GetStartButton ()) { // disable as push button is replaced by TSOP then avoid faulty reset
        Reset (); 
      }*/
      iteration_ct++;

      break;
  }

  bool batteryIsLow = IsBatteryLow (GetBattery ());

  if (batteryIsLow) {
    if (millis () % 1000 > 500) {
      SetLED (LOW);
    } else {
      SetLED (HIGH);
    }
  } else {
    SetLED (HIGH);
  }
}
