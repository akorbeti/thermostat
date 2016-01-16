#include <DHT.h>

#include <Wire.h>
#include <Arduino.h>
#include <Encoder.h>
#include <EEPROM.h>
#include "SimpleTimer.h"

#define RELAY_PIN 5
#define PUSH_ENCODER 2
#define LED_PIN 13
#define FURNACE_LED 12
#define _Digole_Serial_UART_

#define DEBOUNCE_DELAY 5
#define CHANGE_MENU_DELAY 1000
#define DISPLAY_REFRESH_DELAY 5000

#define DISPLAY_MENU 1
#define DISPLAY_NORM 0
#define DISPLAY_TIMER 2

#define FURNACE_BURNING_IN 7


#define BUZZER 6

#include <DigoleSerial.h>
/// Humidity and temp
//#include <TH02_dev.h>


/// DISPLAY DIGOLE
DigoleSerialDisp mydisp(&Serial, 9600); //UART:Arduino UNO: Pin 1(TX)on arduino to RX on module
#define LCDCol 20
#define LCDRow 4
#define DHTPIN 11
#define DHTTYPE DHT22

// Minute timer
SimpleTimer timer;


// Encoder
Encoder myEnc(3, 4);
long oldPosition  = -999;
long lastDebounceMillis = 0;
boolean changedMenu = false;

int suspendForMinutes = 120;
int suspendForHours = 0;
long oldSuspendPosition  = -999;

bool furnaceState = false;

bool drawMenuFlag = true;

unsigned long refreshTimer = 0;

int targetTemp = 21;

DHT dht(DHTPIN, DHTTYPE);

// Display mode


int drawMethod = DISPLAY_NORM;

void setup() {
  Serial.begin(9600);
  delay(150);
  //  TH02.begin();
  dht.begin();
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FURNACE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PUSH_ENCODER, INPUT);
  pinMode(FURNACE_BURNING_IN, INPUT);

  mydisp.begin();
  mydisp.clearScreen(); //CLear screen
  mydisp.setLCDColRow(LCDCol, LCDRow);
  mydisp.displayConfig(0);
  mydisp.disableCursor();
  mydisp.backLightOn();

  EEPROM.get(0, targetTemp);
  EEPROM.get(4, suspendForMinutes);



  // Interrupts
  attachInterrupt(digitalPinToInterrupt(2), setDrawMethod, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(8), furnaceStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderCallback, CHANGE);

  timer.setInterval(60000, everyMinuteCallback);
}

void loop() {
  timer.run();
  switch (drawMethod) {
    case DISPLAY_NORM:
      draw_norm();
      break;
    case DISPLAY_MENU:
      draw_menu();
      break;
    case DISPLAY_TIMER:
      draw_timer();
      break;
    default:
      break;
  }
}

void draw_timer() {
  if (drawMenuFlag) {

    if (suspendForMinutes) {
      mydisp.drawStr(1, 1, "Suspend for         ");
      mydisp.setPrintPos(13, 1);
      mydisp.print(suspendForMinutes / 60, 1  );
      mydisp.drawStr(16, 1, "Hrs ");
    } else {
      mydisp.drawStr(0, 1, "  Normal operation    ");
    }
    drawMenuFlag = false;
  }
}

void draw_menu() {
  if (drawMenuFlag) {
    mydisp.drawStr(2, 1, "Target temp:       ");
    mydisp.setPrintPos(15, 1);
    mydisp.print(targetTemp, 1);
    drawMenuFlag = false;
  }
}

void draw_norm() {
  if (millis() - refreshTimer > DISPLAY_REFRESH_DELAY) {
    refreshTimer = millis();

    //    float h = TH02.ReadHumidity();
    //    // Read temperature as Celsius (the default)
    //    float t = TH02.ReadTemperature();

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (isnan(h) || isnan(t)) {
      mydisp.drawStr(18, 3, "Er");
    } else {
        mydisp.drawStr(18, 3, "Ok");
    }

    if (digitalRead(FURNACE_BURNING_IN)) {
      furnaceState = true;
    } else {
      furnaceState = false;
    }

    // display stuff
    mydisp.drawStr(0, 0, "Current Temp:       ");
    mydisp.setPrintPos(14, 0);
    mydisp.print(t, 1);
    mydisp.print((char)223);

    mydisp.drawStr(0, 1, "Current Hum:        ");
    mydisp.setPrintPos(14, 1);
    mydisp.print(h, 1);
    mydisp.print("%");

    mydisp.drawStr(0, 2, "Target Temp:        ");
    mydisp.setPrintPos(14, 2);
    mydisp.print(targetTemp, 1);
    mydisp.print((char)223);

    if (suspendForMinutes) {
      mydisp.drawStr(0, 3, "Resume in:        ");
      mydisp.setPrintPos(11, 3);
      if (suspendForMinutes > 99) {
        mydisp.print(suspendForMinutes / 60 , 1);
        mydisp.print(":");
        mydisp.print(suspendForMinutes % 60 , 1);
      } else {
        mydisp.print(suspendForMinutes , 1);
        mydisp.drawStr(14, 3, "min ");
      }
    } else {
      mydisp.drawStr(0, 3, "Normal : ");
      if (furnaceState) {
        digitalWrite(FURNACE_LED, HIGH);
        mydisp.print("burning");
      } else {
        digitalWrite(FURNACE_LED, LOW);
        mydisp.print("standby");
      }
    }

    //logic for on-off
    if (t > targetTemp || suspendForMinutes) {
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }

    if (t < targetTemp && !suspendForMinutes) {
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
    }
  }
}

void encoderCallback() {
  if (millis() - lastDebounceMillis > DEBOUNCE_DELAY) {
    if (drawMethod == DISPLAY_MENU ) {
      long newPosition = myEnc.read() / 4;
      if (newPosition > oldPosition) {
        targetTemp ++;
        if (targetTemp > 30) targetTemp = 30;
        oldPosition = newPosition;
        EEPROM.put(0, targetTemp);
        drawMenuFlag = true;
      }

      if (newPosition < oldPosition) {
        targetTemp --;
        if (targetTemp < 12) targetTemp = 12;
        oldPosition = newPosition;
        EEPROM.put(0, targetTemp);
        drawMenuFlag = true;
      }


    } else if (drawMethod == DISPLAY_TIMER) {
      long newSuspendPosition = myEnc.read() / 4;
      if (newSuspendPosition > oldSuspendPosition) {
        suspendForHours ++;
        if (suspendForHours > 20) suspendForHours = 21;
        oldSuspendPosition = newSuspendPosition;
        drawMenuFlag = true;
      }

      if (newSuspendPosition < oldSuspendPosition) {
        suspendForHours --;
        if (suspendForHours < 1) suspendForHours = 0;
        oldSuspendPosition = newSuspendPosition;
        drawMenuFlag = true;
      }

      suspendForMinutes = suspendForHours * 60;
      EEPROM.put(4, suspendForMinutes);
    }
    lastDebounceMillis = millis();

  }
}

void setDrawMethod() {
  if (millis() - lastDebounceMillis > CHANGE_MENU_DELAY) {
    beep();
    if (drawMethod == DISPLAY_NORM) {
      drawMethod = DISPLAY_MENU;
      drawMenuFlag = true;
    } else if (drawMethod == DISPLAY_MENU) {
      drawMethod = DISPLAY_TIMER;
      drawMenuFlag = true;
    } else if (drawMethod == DISPLAY_TIMER) {
      drawMethod = DISPLAY_NORM;
    }

    lastDebounceMillis = millis();

    mydisp.clearScreen();

  }
}


void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
}


void everyMinuteCallback() {
  if (suspendForMinutes > 0) {
    suspendForMinutes--;
    EEPROM.put(4, suspendForMinutes);
    if (suspendForMinutes == 0) {
      beep();
    }
  }
}


void furnaceStateChange() {

}
