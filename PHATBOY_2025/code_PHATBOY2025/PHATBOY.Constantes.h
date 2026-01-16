#ifdef PhatboyBlue
#define POSCLOSE         20
#define POSOPEN          0
#endif


#ifdef PhatboyViolet
#define POSCLOSE         0
#define POSOPEN          20

#endif 

#define MINWHITEVALUE    200
#define MAXDISTVALUE     350
#define MAXFULLPOWER     70
#define MINBATTERYVALUE  7.4

#define WAITTIME         5000
//#define WAITTIME         3000

#define BACKTIME         900
#define TIMEOUT          100
#define FOCUSTIME        400
#define MAXDISTDIFF      42

/*----------------------------------------
              Acquisition
----------------------------------------*/

#define OUT_NMOS    A0
#define IN_BATTERY  A6
#define IN_BTN      8
#define IN_IRBACK   A1  //CAPTAR
#define IN_IRRIGHT  A2  //CAPTLAVD
#define IN_IRLEFT   A3  //CAPTLAVG
#define XSHUT       5

#define SDA         A4
#define SCL         A5

/*----------------------------------------
                  Action
----------------------------------------*/
#define LED_B 6
#define OUT_PWM0   12 //AIN2 // PWM
#define OUT_PWM1   11 //AIN1
#define OUT_PWM2   13  //BIN2 // PWM
#define OUT_PWM3   3  //BIN1

#define OUT_SERVO  9
