#pragma once
#include "RATS.h"
#include "Pololu3piPlus32U4IMU.h"

class IntertialMeasurementUnit {
private:
  float xOffset;
  float yOffset;
  float zOffset;
  float pitchOffset;
  float rollOffset;

  
public:
  Pololu3piPlus32U4::IMU myIMU;
  IntertialMeasurementUnit() : xOffset(0.0), yOffset(0.0), zOffset(0.0),
  pitchOffset(0.0), rollOffset(0.0) {}

  void calibrate() {
    myIMU.readMag();
    myIMU.readAcc();
    xOffset = myIMU.m.x;
    yOffset = myIMU.m.y;
    zOffset = myIMU.m.z;

    float accelX = myIMU.a.x;
    float accelY = myIMU.a.y;
    float accelZ = myIMU.a.z;

    float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    float normX = accelX / magnitude;
    float normY = accelY / magnitude;
    float normZ = accelZ / magnitude;

    pitchOffset = calculatePitch(normX, normY, normZ);
    rollOffset = calculateRoll(normX, normY, normZ);
  }

  Option<Vec3<float>> foundAnamoly() {
    myIMU.readMag();
    int16_t magX = myIMU.m.x;
    int16_t magY = myIMU.m.y;
    int16_t magZ = myIMU.m.z;

    float magneticStrength = calculateMagneticStrength(magX - xOffset, magY - yOffset, magZ - zOffset);

    if (magneticStrength <= MAG_THRESHOLD) {
      return Option<Vec3<float>>();
    }

    return Option<Vec3<float>>({magX - xOffset, magY - yOffset, magZ - zOffset});
  }

  float getStrength() {
    myIMU.readMag();
    int16_t magX = myIMU.m.x;
    int16_t magY = myIMU.m.y;
    int16_t magZ = myIMU.m.z;

    float magneticStrength = calculateMagneticStrength(magX - xOffset, magY - yOffset, magZ - zOffset);

    return magneticStrength;
  }

  inline float calculateMagneticStrength(int16_t x, int16_t y, int16_t z) {
    return sqrt((float)x * x + (float)y * y + (float)z * z);
  }

  Vec2<float> getOrientation() {
    myIMU.readAcc();
    float accelX = myIMU.a.x;
    float accelY = myIMU.a.y;
    float accelZ = myIMU.a.z;

    float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    float normX = accelX / magnitude;
    float normY = accelY / magnitude;
    float normZ = accelZ / magnitude;

    float pitch = calculatePitch(normX, normY, normZ) - pitchOffset;
    float roll = calculateRoll(normX, normY, normZ) - rollOffset;

    return {pitch, roll};
  }

  inline float calculatePitch(float x, float y, float z) {
    return atan2(-x, sqrt(y * y + z * z)) * 180.0 / M_PI;
  }

  inline float calculateRoll(float x, float y, float z) {
    return atan2(y, sqrt(x * x + z * z)) * 180.0 / M_PI;
  }
};