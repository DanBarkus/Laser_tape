#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// This code uses an accelerometer and a ToF distance sensor to compute the real world distance that exists before a starting and ending point in space
// It's essentially a contact-less tape measure

VL53L1X sensor;

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

// Set sample rate as fast as we can 
#define BNO055_SAMPLERATE_DELAY_MS (33)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// init button in and laser power pins
static int trigger1 = 11;
static int trigger2 = 10;
static int laser = 13;

// Arm length in mm, used to fine tune accuracy for now
static float armLength = 350;

// Vars for computing distance
float startAngle = 0.0;
float deltaX = 0.0;
float startDistance = 0.0;
float endDistance = 0.0;
float calcDistance = 0.0;

// State booleans
bool firstTrig = true;
bool trigging = false;

void setup(void)
{
  pinMode(trigger1, INPUT_PULLUP);
  pinMode(trigger2, INPUT_PULLUP);
  pinMode(laser, OUTPUT);

  Wire.begin();

  Serial.begin(9600);
  alpha4.begin(0x70);  // pass in the address
  alpha4.setBrightness(4);
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

// See if we're pulling either stage of the trigger
  int trig1 = digitalRead(trigger1);
  int trig2 = digitalRead(trigger2);

// When the trigger is half pulled we read and display the distance
  if (trig1 == 0) {
    float distance = sensor.read();
    // When the trigger is pulled fully, we continually capture the distance and angle until released
    if (trig2 == 0) {
      sensors_event_t event;
      bno.getEvent(&event);
      trigging = true;

      // Store the start angle and distance on our initial reading (first side of the triangle)
      if (firstTrig) {
        calcDistance = 0;
        startAngle = event.orientation.x;
        startDistance = distance + armLength;
        // this was supposed to prevent the jump from 359 degrees to 0 degrees from being present, it instead breaks readings at 180 and 0 degrees
        if (startAngle >= 180) {
          startAngle = 360 - startAngle;
        }
        firstTrig = false;
      }
      float orientX = event.orientation.x;
      if (orientX >= 180) {
        orientX = 360 - orientX;
      }
      // find our total angular travel
      deltaX = orientX - startAngle;
      // trig to find side A using sides B and C and angle a 
      calcDistance = sqrt(sq(startDistance) + sq(endDistance) - 2*startDistance*endDistance * cos(radians(deltaX)));
      // Show the new distance in fractional feet
      updateTextDisplay(calcDistance * 0.0032808416);
      // This would show inches
      //updateTextDisplay(calcDistance * 0.0393701);
    }
    else {
      // When we stop pulling the trigger all the way down, we capture the last distance and angle 
      if (trigging) {
        endDistance = distance + armLength;
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
  // show the last computed A distance when we're not doing anything
  else {
    updateTextDisplay(calcDistance * 0.0032808416);
    //updateTextDisplay(calcDistance * 0.0393701);
    digitalWrite(laser, LOW);
  }
  alpha4.writeDisplay();
}

// This displays floats on a 4 digit 7 or 14 segment display
void updateTextDisplay(float score)
{
  Serial.println(score);
  // Multiply by 10 because we have to deal with int values and don't want to lose our decimal value
  String stringScore = String(int(score * 10));
  // Check if we need to add a leading 0
  if (stringScore.toInt() < 10)
  {
    stringScore = "0" + stringScore;
  }
  // Add 0's for remaining length
  for (int j = 0; j <= (4 - stringScore.length()); j++)
  {
    String temp = "0";
    temp += stringScore;
    stringScore = temp;
  }
  // Write each digit to its corresponding display with a decimal allowing for display of a tenths place 
  for (int i = 0; i < 4; i++)
  {
    alpha4.writeDigitAscii(0, stringScore[0]);
    alpha4.writeDigitAscii(1, stringScore[1]);
    alpha4.writeDigitAscii(2, stringScore[2], "drawDots");
    alpha4.writeDigitAscii(3, stringScore[3]);
  }
}
