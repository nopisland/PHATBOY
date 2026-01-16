
/*----------------------------------------
             Initialization
  ----------------------------------------*/

void InitPins () {
  pinMode (IN_IRLEFT  ,  INPUT  );
  pinMode (IN_IRBACK  ,  INPUT  );
  pinMode (IN_IRRIGHT ,  INPUT  );
  pinMode (OUT_NMOS   ,  OUTPUT );
  pinMode (IN_BATTERY ,  INPUT  );
  pinMode (IN_BTN     ,  INPUT_PULLUP);
  pinMode (XSHUT      ,  OUTPUT );
  pinMode (OUT_PWM0   ,  OUTPUT );
  pinMode (OUT_PWM1   ,  OUTPUT );
  pinMode (OUT_PWM2   ,  OUTPUT );
  pinMode (OUT_PWM3   ,  OUTPUT );
  pinMode (OUT_RGB_R  ,  OUTPUT );
  pinMode (OUT_RGB_G  ,  OUTPUT );
  pinMode (OUT_RGB_B  ,  OUTPUT );
  pinMode (OUT_LED    ,  OUTPUT );
}

void InitSensors() {

  // Changing address of sensor 1

  digitalWrite(XSHUT, LOW);
  delay(1);
  Wire.begin();
  sensor1.init();
  sensor1.setAddress(0x0a);
  sensor1.setTimeout(500);
  digitalWrite(XSHUT, HIGH);

  sensor.init();
  sensor.setTimeout(500);

  sensor.setMeasurementTimingBudget(20000);
  sensor1.setMeasurementTimingBudget(20000);
  sensor.startContinuous();
  sensor1.startContinuous();
}

void Calibrate () {

  // Calibration of distance sensors;

  unsigned long distance = 65535;
  unsigned long distance1 = 65535;

  distance = sensor.readRangeContinuousMillimeters();
  distance1 = sensor1.readRangeContinuousMillimeters();

  if (distance < 25 && distance1 < 25) {
    offset = distance - distance1;
  }

  // Calibration of line sensors;

  mesuredBlack = (analogRead (IN_IRLEFT) + analogRead (IN_IRBACK) + analogRead (IN_IRRIGHT)) / 3;

  digitalWrite (OUT_NMOS, LOW);
  //Serial.print ("Eteint = ");
  //Serial.println (analogRead (IN_IRLEFT));
  int blackOff = analogRead (IN_IRLEFT);

  delay (50);

  digitalWrite (OUT_NMOS, HIGH);
  //Serial.print ("Allume = ");
  //Serial.println (analogRead (IN_IRLEFT));
  int blackOn = analogRead (IN_IRLEFT);

  blackDiff += abs (blackOn - blackOff);
}

void Reset () {
  SetArm (LOW);
  SetLED (HIGH) ;
  SetRGB (255, 255, 0); // YELLOW
  etat = 0;
  SetSpeed (0, 0);
  delay (500);
}

/*----------------------------------------
              Acquisition
  ----------------------------------------*/

bool GetStartButton_orig () {
  if (digitalRead (IN_BTN) == 0) {  
    return HIGH;
  } else {
    return LOW;
  }
}

bool GetStartButton () {
  if (digitalRead (IN_BTN) == 0) {
    delay(10);
    return (digitalRead(IN_BTN) == 0);
  } else {
    return LOW;
  }
}

float GetBattery () {
  float voltage = analogRead (IN_BATTERY) * 10.0 / 1024.0;
  //Serial.print ("Battery = ");
  //Serial.println (voltage);
  return voltage;
}

void GetLines () {

  if (analogRead (IN_IRLEFT) < mesuredBlack - blackDiff && millis () - timeBack > 200) {
    //Serial.println ("BLANC GAUCHE");
    lLine = HIGH;
    rLine = LOW;
    bLine = LOW;
    avoiding = HIGH;
    timeBack = millis ();
    SetSpeed (-255, -255);
  }

  if (analogRead (IN_IRRIGHT) < mesuredBlack - blackDiff && millis () - timeBack > 200) {
    //Serial.println ("BLANC DROIT");
    lLine = LOW;
    rLine = HIGH;
    bLine = LOW;
    avoiding = HIGH;
    timeBack = millis ();
    SetSpeed (-255, -255);
  }

  if (analogRead (IN_IRBACK) < mesuredBlack - blackDiff && millis () - timeBack > 200) {
    //Serial.println ("BLANC ARRIERE");
    lLine = LOW;
    rLine = LOW;
    bLine = HIGH;
    avoiding = HIGH;
    timeBack = millis ();
    SetSpeed (255, 255);
  }
}

/*----------------------------------------
                Processing
  ----------------------------------------*/


bool IsBatteryLow (float level) {
  if (level < MINBATTERYVALUE) {
    return HIGH;
  } else {
    return LOW;
  }
}

/*----------------------------------------
             Advanced actions
  ----------------------------------------*/

void AvoidLines () {
  int backtime = BACKTIME * 6 / GetBattery ();

  if (lLine == HIGH) {
    if (millis () - timeBack < 4 * BACKTIME / 8) {
      SetSpeed (-255, -255); //Recul
    } else if (millis () < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed (255, -255); //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW; //Stop
      rLine = LOW; //Stop
      bLine = LOW; //Stop
      SetSpeed (255, 255);
    }
  }

  else if (rLine == HIGH) {
    if (millis () - timeBack < 4 * BACKTIME / 8) {
      SetSpeed (-255, -255); //Recul
    } else if (millis () < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed (-255, 255); //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW; //Stop
      rLine = LOW; //Stop
      bLine = LOW; //Stop
      SetSpeed (255, 255);
      avoiding = LOW;
    }
  }

  else if (bLine == HIGH) {
    if (millis () - timeBack < 4 * BACKTIME / 8) {
      SetSpeed (255, 255); //avant
    } else if (millis () < timeBack + 6.5 * BACKTIME / 8) {
      SetSpeed (-255, 255); //Pivot
      avoiding = LOW;
    } else {
      lLine = LOW; //Stop
      rLine = LOW; //Stop
      bLine = LOW; //Stop
      SetSpeed (255, 255);
    }
  }
}

void FocusRobot () {
  int timeout = 100;
  unsigned long temps = 0;
  unsigned long distance = 65535;
  unsigned long distance1 = 65535;

  distance = sensor.readRangeContinuousMillimeters();
  distance1 = sensor1.readRangeContinuousMillimeters() + offset;

  //Tuple(distance, distance1);

  if (distance < MAXFULLPOWER && distance1 < MAXFULLPOWER) {
    SetSpeed (255, 255);
    SetRGB (255, 0, 0);  // BLUE
  }

  else if (distance < MAXDISTVALUE || distance1 < MAXDISTVALUE) {
    if (distance > distance1) {
      SetSpeed (0, 255);
    } else {
      SetSpeed (255, 0);
    }

    if (millis () % 200 > 100) {
      SetRGB (0, 0, 0);  // BLACK
    } else {
      SetRGB (0, 255, 255);  // CYAN
    }

  } else if (!lLine && !rLine && !bLine) {
    SetRGB (0, 0, 255);  // BLUE
  }
}

/*----------------------------------------
             Basic actions
  ----------------------------------------*/

void SetSpeed (short left, short right) {
  if (left == 0) {
    analogWrite (OUT_PWM0, 0);
    analogWrite (OUT_PWM1, 0);
  } else if (left < 0) {
    analogWrite(OUT_PWM0, -left);
    analogWrite(OUT_PWM1, 0);
  } else {
    analogWrite(OUT_PWM0, 0);
    analogWrite(OUT_PWM1, left);
  }

  if (right == 0) {
    analogWrite (OUT_PWM2, 0);
    analogWrite (OUT_PWM3, 0);
  } else if (right < 0) {
    analogWrite(OUT_PWM2, 0);
    analogWrite(OUT_PWM3, -right);
  } else {
    analogWrite(OUT_PWM2, right);
    analogWrite(OUT_PWM3, 0);
  }
}

void SetArm (bool state) {
  if (state == HIGH) {
    servo.write (POSHIGH);
  } else {
    servo.write (POSLOW);
  }
}

void SetRGB (short r, short g, short b) {
  digitalWrite (OUT_RGB_R, (r < 255) ? HIGH : LOW);
  digitalWrite (OUT_RGB_G, (g < 255) ? HIGH : LOW);
  analogWrite  (OUT_RGB_B, 255 - b);
}

void SetLED (bool state) {
  digitalWrite (OUT_LED, !state);
}
