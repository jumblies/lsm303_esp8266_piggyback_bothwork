#include <Arduino.h>

//Although the piggyback works, the ground must be supplemented to get accurate accel readings.

// I2C interface by default
//
#include "Wire.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"

// #define DEBUG 1 in SparkFunLSM303C.h turns on debugging statements.
// Redefine to 0 to turn them off.

long lastDisplayTime;

LSM303C myIMU;

//Define LED indicator Pins
const int  northPin = LED_BUILTIN;//Wemos D1 Builtin LED
const int southPin = 12;


float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;

float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;

//Nonblocking code
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 500;

boolean northState, southState;


void setup()
{
  Serial.begin(115200);
  Wire.begin(D6, D7);
  pinMode(D8, LOW);
  if (myIMU.begin() != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while (1);
  }
  pinMode(northPin, OUTPUT);
  digitalWrite(northPin, HIGH);
  digitalWrite(southPin, HIGH);

  lastDisplayTime = millis();

}

void loop()
{ unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    Serial.println("**************************************************");
    Serial.println("\tMagnetometer:\tAccelerometer:");
    Serial.print(" X = \t");
    Serial.print(myIMU.readMagX(), 4);
    Serial.print("\t\t");
    Serial.println(myIMU.readAccelX(), 4);
    Serial.print(" Y = \t");
    Serial.print(myIMU.readMagY(), 4);
    Serial.print("\t\t");
    Serial.println(myIMU.readAccelY(), 4);
    Serial.print(" Z = \t");
    Serial.print(myIMU.readMagZ(), 4);
    Serial.print("\t\t");
    Serial.println(myIMU.readAccelZ(), 4);
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F = ");
    Serial.println(myIMU.readTempF(), 4);


    //Calibration Code
    Serial.println("**************************************************");
    float AccelX = (myIMU.readAccelX());
    if (AccelX < AccelMinX)AccelMinX = AccelX;
    if (AccelX > AccelMaxX)AccelMaxX = AccelX;
    Serial.print(AccelMinX, 4);
    Serial.print("\t\t");
    Serial.println(AccelMaxX, 4);

    float AccelY = myIMU.readAccelY();
    if (AccelY < AccelMinY) AccelMinY = AccelY;
    if (AccelY > AccelMaxY) AccelMaxY = AccelY;
    Serial.print(AccelMinY, 4);
    Serial.print("\t\t");
    Serial.println(AccelMaxY, 4);

    float AccelZ = myIMU.readAccelZ();
    if (AccelZ < AccelMinZ) AccelMinZ = AccelZ;
    if (AccelZ > AccelMaxZ) AccelMaxZ = AccelZ;
    Serial.print(AccelMinZ, 4);
    Serial.print("\t\t");
    Serial.println(AccelMaxZ, 4);

    float MagX = myIMU.readMagX();
    if (MagX < MagMinX) MagMinX = (MagX);
    if (MagX > MagMaxX) MagMaxX = (MagX);
    Serial.print(MagMinX, 4);
    Serial.print("\t\t");
    Serial.println(MagMaxX, 4);


    //compute compass heading
    float Pi = 3.14159;

    // Calculate the angle of the vector y,x
    float heading = (atan2(myIMU.readMagY(), myIMU.readMagX()) * 180) / Pi;

    // Normalize to 0-360
    if (heading < 0)
    {
      heading = 360 + heading;
    }
    Serial.print("Compass Heading: ");
    Serial.println(360 - heading);
    if (heading > 345 || heading < 15) {
      if (northState == HIGH) {
        northState = LOW;
      } else {
        northState = HIGH;
      }
      digitalWrite(northPin, northState);
    }
    else {
      digitalWrite(northPin, HIGH);
    }
    if (heading > 165 && heading < 195) {
      if (southState == LOW) {
        southState = HIGH;
      } else {
        southState = LOW;
      }
      digitalWrite(southPin, southState);
    }
    else {
      digitalWrite(southPin, LOW);
    }


  }
  delay(1000);
}
