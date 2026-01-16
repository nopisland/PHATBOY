

      #include "Arduino.h"
/*----------------------------------------
             Initialization
----------------------------------------*/

void InitPins ();

void InitSensors();

void Calibrate ();

void Reset ();

/*----------------------------------------
              Acquisition
----------------------------------------*/

bool GetStartButton ();

float GetBattery ();

void GetLines ();

/*----------------------------------------
                Processing
----------------------------------------*/


bool IsBatteryLow (float);

/*----------------------------------------
                  Action
----------------------------------------*/
  

void SetSpeed (short, short);

void SetArm (bool);

void SetRGB (short , short , short );

void SetLED (bool);

/*----------------------------------------
             Advanced actions
  ----------------------------------------*/
void AvoidLines ();

void FocusRobot();
/*----------------------------------------
                  DEBUG
----------------------------------------*/

void Debug_Aquisition(void); 
void Debug_Action(void);
