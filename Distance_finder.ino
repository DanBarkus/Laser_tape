#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


VL53L1X sensor;

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

#define BNO055_SAMPLERATE_DELAY_MS (33)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

static int trigger1 = 11;
static int trigger2 = 10;
static int laser = 13;

bool firstTrig = true;
bool trigging = false;
float startAngle = 0.0;
float deltaX = 0.0;
float startDistance = 0.0;
float endDistance = 0.0;
float calcDistance = 0.0;

void setup(void)
{
  pinMode(trigger1, INPUT_PULLUP);
  pinMode(trigger2, INPUT_PULLUP);
  pinMode(laser, OUTPUT);

  Wire.begin();

  Serial.begin(9600);
  alpha4.begin(0x70);  // pass in the address
  alpha4.setBrightness(2);
  alpha4.clear();
  alpha4.writeDigitAscii(2, 'O');
  alpha4.writeDigitAscii(3, 'K');
  alpha4.writeDisplay();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  bno.setExtCrystalUse(true);

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(33000);

  sensor.startContinuous(33);

}

void loop(void)
{

  int trig1 = digitalRead(trigger1);
  int trig2 = digitalRead(trigger2);

  if (trig1 == 0) {
    float distance = sensor.read();
    if (trig2 == 0) {
      sensors_event_t event;
      bno.getEvent(&event);
      trigging = true;

      if (firstTrig) {
        startAngle = event.orientation.x;
        startDistance = distance;
        if (startAngle >= 180) {
          startAngle = 360 - startAngle;
        }
        firstTrig = false;
      }
      float orientX = event.orientation.x;
      if (orientX >= 180) {
        orientX = 360 - orientX;
      }
      deltaX = orientX - startAngle;
      calcDistance = sqrt(sq(startDistance) + sq(endDistance) - 2*startDistance*endDistance * cos(radians(deltaX)));
      updateTextDisplay(calcDistance * 0.0032808416);
    }
    else {
      if (trigging) {
        endDistance = distance;
        trigging = false;
      };
      firstTrig = true;
    };
    
    Serial.print("Distance(mm): ");
    Serial.print(distance);

    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;
    Serial.print("\tDistance(ft): ");
    Serial.print(distanceFeet, 2);
    if (trig2 > 0){
      updateTextDisplay(distanceFeet);
    }

    Serial.println();
    digitalWrite(laser, HIGH);
  }
  else {
    updateTextDisplay(calcDistance * 0.0032808416);
    digitalWrite(laser, LOW);
  }
  alpha4.writeDisplay();
}

void updateTextDisplay(float score)
{
  Serial.println(score);
  String stringScore = String(int(score * 10));
  if (stringScore.toInt() < 10)
  {
    stringScore = "0" + stringScore;
  }
  for (int j = 0; j <= (4 - stringScore.length()); j++)
  {
    String temp = "0";
    temp += stringScore;
    stringScore = temp;
  }
  for (int i = 0; i < 4; i++)
  {
    alpha4.writeDigitAscii(0, stringScore[0]);
    alpha4.writeDigitAscii(1, stringScore[1]);
    alpha4.writeDigitAscii(2, stringScore[2], "drawDots");
    alpha4.writeDigitAscii(3, stringScore[3]);
  }
}
