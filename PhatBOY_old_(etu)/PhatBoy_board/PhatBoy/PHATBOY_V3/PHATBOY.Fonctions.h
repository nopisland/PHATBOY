
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
  
void AvoidLines ();

void SetSpeed (short, short);

void SetArm (bool);

void SetRGB (short , short , short );

void SetLED (bool);
