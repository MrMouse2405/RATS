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

  Pololu3piPlus32U4::IMU ratsIMU;
public:
  IntertialMeasurementUnit() : xOffset(0.0), yOffset(0.0), zOffset(0.0),
  pitchOffset(0.0), rollOffset(0.0) {}

  void calibrate() {
    ratsIMU.readMag();
    ratsIMU.readAcc();
    xOffset = ratsIMU.m.x;
    yOffset = ratsIMU.m.y;
    zOffset = ratsIMU.m.z;

    float accelX = ratsIMU.a.x;
    float accelY = ratsIMU.a.y;
    float accelZ = ratsIMU.a.z;

    float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    float normX = accelX / magnitude;
    float normY = accelY / magnitude;
    float normZ = accelZ / magnitude;

    pitchOffset = calculatePitch(normX, normY, normZ);
    rollOffset = calculateRoll(normX, normY, normZ);
  }

  Option<Vec3<float>> foundAnamoly() {
    ratsIMU.readMag();
    int16_t magX = ratsIMU.m.x;
    int16_t magY = ratsIMU.m.y;
    int16_t magZ = ratsIMU.m.z;

    float magneticStrength = calculateMagneticStrength(magX - xOffset, magY - yOffset, magZ - zOffset);

    if (magneticStrength <= MAG_THRESHOLD) {
      return Option<Vec3<float>>();
    }

    return Option<Vec3<float>>({magX - xOffset, magY - yOffset, magZ - zOffset});
  }

  float getStrength() {
    ratsIMU.readMag();
    int16_t magX = ratsIMU.m.x;
    int16_t magY = ratsIMU.m.y;
    int16_t magZ = ratsIMU.m.z;

    float magneticStrength = calculateMagneticStrength(magX - xOffset, magY - yOffset, magZ - zOffset);

    return magneticStrength;
  }

  float calculateMagneticStrength(int16_t x, int16_t y, int16_t z) {
    return sqrt((float)x * x + (float)y * y + (float)z * z);
  }

  Vec2<float> getOrientation() {
    ratsIMU.readAcc();
    float accelX = ratsIMU.a.x;
    float accelY = ratsIMU.a.y;
    float accelZ = ratsIMU.a.z;

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