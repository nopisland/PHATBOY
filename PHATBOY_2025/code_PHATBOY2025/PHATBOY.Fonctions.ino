#include "PHATBOY.Fonctions.h"
/*----------------------------------------
             Initialization
  ----------------------------------------*/

void InitPins() {
  pinMode(IN_IRLEFT, INPUT);
  pinMode(IN_IRBACK, INPUT);
  pinMode(IN_IRRIGHT, INPUT);
  pinMode(OUT_NMOS, OUTPUT);
  pinMode(IN_BATTERY, INPUT);
  pinMode(IN_BTN, INPUT_PULLUP);
  pinMode(XSHUT, OUTPUT);

  pinMode(LED_B, OUTPUT);
  pinMode(OUT_PWM0, OUTPUT);
  pinMode(OUT_PWM1, OUTPUT);
  pinMode(OUT_PWM2, OUTPUT);
  pinMode(OUT_PWM3, OUTPUT);
}

void InitSensors() {

  // Changing address of sensor 1


  Wire.begin();
//SHUTDOWN sensor 1
  digitalWrite(XSHUT, HIGH);

  delay(10);


  delay(10);
  sensor.init();
  sensor.setAddress(0x0B);
  sensor.setTimeout(500);
//REVIVE SENSOR 1
  digitalWrite(XSHUT, LOW);
  delay(10);
  sensor1.init();
  sensor1.setAddress(0x0a);
  sensor1.setTimeout(500);



  sensor.setMeasurementTimingBudget(20000);
  sensor1.setMeasurementTimingBudget(20000);
  sensor.startContinuous();
  sensor1.startContinuous();
}

void Calibrate() {

  // Calibration of distance sensors;

  unsigned long distance = 65535;
  unsigned long distance1 = 65535;

  distance = sensor.readRangeContinuousMillimeters();
  //  distance1 = sensor1.readRangeContinuousMillimeters();

  if (distance < 25 && distance1 < 25) {
    offset = distance - distance1;
  }

  // Calibration of line sensors;

  mesuredBlack = (analogRead(IN_IRLEFT) + analogRead(IN_IRBACK) + analogRead(IN_IRRIGHT)) / 3;

  digitalWrite(OUT_NMOS, LOW);
  //Serial.print ("Eteint = ");
  //Serial.println (analogRead (IN_IRLEFT));
  int blackOff = analogRead(IN_IRLEFT);

  delay(50);

  digitalWrite(OUT_NMOS, HIGH);
  //Serial.print ("Allume = ");
  //Serial.println (analogRead (IN_IRLEFT));
  int blackOn = analogRead(IN_IRLEFT);

  blackDiff += abs(blackOn - blackOff);
}

void Reset() {
  SetArm(LOW);
  SetLED(HIGH);
  SetRGB(255, 255, 0);  // YELLOW
  etat = 0;
  SetSpeed(0, 0);
  delay(500);
}

/*----------------------------------------
              Acquisition
  ----------------------------------------*/

bool GetStartButton_orig() {
  if (digitalRead(IN_BTN) == 0) {
    return HIGH;
  } else {
    return LOW;
  }
}

bool GetStartButton() {
  if (digitalRead(IN_BTN) == 0) {
    delay(10);
    return (digitalRead(IN_BTN) == 0);
  } else {
    return LOW;
  }
}

float GetBattery() {
  float voltage = analogRead(IN_BATTERY) * 10.0 / 1024.0;
  //Serial.print ("Battery = ");
  //Serial.println (voltage);
  return voltage;
}

void GetLines() {

  if (analogRead(IN_IRLEFT) < mesuredBlack - blackDiff && millis() - timeBack > 200) {
    //Serial.println ("BLANC GAUCHE");
    lLine = HIGH;
    rLine = LOW;
    bLine = LOW;
    avoiding = HIGH;
    timeBack = millis();
    SetSpeed(-255, -255);
  }

  if (analogRead(IN_IRRIGHT) < mesuredBlack - blackDiff && millis() - timeBack > 200) {
    //Serial.println ("BLANC DROIT");
    lLine = LOW;
    rLine = HIGH;
    bLine = LOW;
    avoiding = HIGH;
    timeBack = millis();
    SetSpeed(-255, -255);
  }

  if (analogRead(IN_IRBACK) < mesuredBlack - blackDiff && millis() - timeBack > 200) {
    //Serial.println ("BLANC ARRIERE");
    lLine = LOW;
    rLine = LOW;
    bLine = HIGH;
    avoiding = HIGH;
    timeBack = millis();
    SetSpeed(255, 255);
  }
}

/*----------------------------------------
                Processing
  ----------------------------------------*/


bool IsBatteryLow(float level) {
  if (level < MINBATTERYVALUE) {
    return HIGH;
  } else {
    return LOW;
  }
}

/*----------------------------------------
             Advanced actions
  ----------------------------------------*/

void AvoidLines() {
  int backtime = BACKTIME * 6 / GetBattery();

  if (lLine == HIGH) {
    if (millis() - timeBack < 4 * BACKTIME / 8) {
      SetSpeed(-255, -255);  //Recul
    } else if (millis() < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed(255, -255);  //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW;  //Stop
      rLine = LOW;  //Stop
      bLine = LOW;  //Stop
      SetSpeed(255, 255);
    }
  }

  else if (rLine == HIGH) {
    if (millis() - timeBack < 4 * BACKTIME / 8) {
      SetSpeed(-255, -255);  //Recul
    } else if (millis() < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed(-255, 255);  //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW;  //Stop
      rLine = LOW;  //Stop
      bLine = LOW;  //Stop
      SetSpeed(255, 255);
      avoiding = LOW;
    }
  }

  else if (bLine == HIGH) {
    if (millis() - timeBack < 4 * BACKTIME / 8) {
      SetSpeed(255, 255);  //avant
    } else if (millis() < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed(-255, 255);  //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW;  //Stop
      rLine = LOW;  //Stop
      bLine = LOW;  //Stop
      SetSpeed(255, 255);
    }
  }
}

void FocusRobot() {
  int timeout = 100;
  unsigned long temps = 0;
  unsigned long distance = 65535;
  unsigned long distance1 = 65535;

  distance = sensor.readRangeContinuousMillimeters();
  distance1 = sensor1.readRangeContinuousMillimeters() + offset;

  //Tuple(distance, distance1);

  if (distance < MAXFULLPOWER && distance1 < MAXFULLPOWER) {
    SetSpeed(255, 255);

  }

  else if (distance < MAXDISTVALUE || distance1 < MAXDISTVALUE) {
    if (distance > distance1) {
      SetSpeed(0, 255);
    } else {
      SetSpeed(255, 0);
    }

    if (millis() % 200 > 100) {

    } else {
    }

  } else if (!lLine && !rLine && !bLine) {
  }
}

/*----------------------------------------
             Basic actions
  ----------------------------------------*/

void SetSpeed(short left, short right) {
  if (left == 0) {
    analogWrite(OUT_PWM0, 0);
    analogWrite(OUT_PWM1, LOW);
   
  } else if (left < 0) {
    analogWrite(OUT_PWM0, -left);
    digitalWrite(OUT_PWM1, LOW);
 
  } else {
    analogWrite(OUT_PWM0, left);
    digitalWrite(OUT_PWM1, HIGH);
  
  }

  if (right == 0) {
    Serial.println("here");
    analogWrite(OUT_PWM2, 0);
    analogWrite(OUT_PWM3, LOW);
 
  } else if (right < 0) {
    Serial.println("here2");
    analogWrite(OUT_PWM2, -right);
    digitalWrite(OUT_PWM3, HIGH);
    
  } else {
    Serial.println("here3");
    analogWrite(OUT_PWM2, right);
    digitalWrite(OUT_PWM3, LOW);
  
  }
}

void SetArm(bool state) {
  if (state == HIGH) {
    servo.write(POSOPEN);
  } else {
    servo.write(POSCLOSE);
  }
}


/*----------------------------------------
                  DEBUG
  ----------------------------------------*/
void Debug_Aquisition(void) {
  int debug_ligne_AR = analogRead(IN_IRBACK);
  int debug_ligne_GAUCHE = analogRead(IN_IRLEFT);
  int debug_ligne_DROITE = analogRead(IN_IRRIGHT);
  int debug_IR_receiver = irDecoder.dataAvailable(irData) ;  // define with de lib TODO
  int debug_Start_button = GetStartButton();
  int debug_state_batterie =  analogRead(A6);
  int debug_distance_sensor0 = sensor.readRangeContinuousMillimeters();
  int debug_distance_sensor1 = sensor1.readRangeContinuousMillimeters();

  Serial.print("Ligne sensor :   ");
  Serial.print("||AR ==> ");
  Serial.print(debug_ligne_AR);
  Serial.print("||LEFT ==> ");
  Serial.print(debug_ligne_GAUCHE );
  Serial.print("||RIGHT ==> ");
  Serial.println(debug_ligne_DROITE );
  Serial.print("IR_Receiver ==> ");
  Serial.print(debug_IR_receiver);
  Serial.print("  || start Button");
  Serial.print(debug_Start_button);    
  Serial.print("  || Batterie");
  Serial.print(debug_state_batterie);  
  Serial.print("  || Distance0   ");
  Serial.print(debug_distance_sensor0); 
  Serial.print("  || Distance1   ");
  Serial.println(debug_distance_sensor1);
    

}

void Debug_Action(void) {
  //Mouvement souhaité


  SetSpeed(255, 150); //Tout Droit 1s
  Serial.println("Tout Droit");
  delay(1000);
  SetSpeed(0, 150); //Gauche 1S
  Serial.println("Gauche");
  delay(1000);
  SetSpeed(255, 0); //Droite 2s
  Serial.println("Droite");
  delay(2000);/*
 // SetSpeed(255, 255); //Tout Droit 1s
 // Serial.println("Tout Droit");
 // delay(1000);
  SetSpeed(-255, -255); //Marche Arrière 1s
  Serial.println("Marche Arrière");
  delay(1000);
  SetSpeed(0, 0); //Stop
  Serial.println("Stop");
  delay(1000);
*/

}
