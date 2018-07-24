#include "quaternionFilters.h"
#include "MPU9250.h"

/* 
 *  Adaptation of MPU9250 Basic Example Code by Kris Winer (04/01/2014)
  by: Yoonyoung (Jamie) Cho @ (07/24/2018)
  MIT license

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  Hardware setup:
  MPU9250 Breakout --------- Arduino (Due)
  VDD ---------------------- 3.3V
  GND ---------------------- GND
  SDA ----------------------- SCL
  SCL ----------------------- SDA
*/
struct IMUHandle {
  int intPin;
  bool calibrate;
  bool ahrs;

  float qx, qy, qz, qw;

  MPU9250 myIMU;

  IMUHandle(const int intPin):
    intPin(intPin), calibrate(false), ahrs(false) {
    // todo : handle calibration requests

  }
  void setup() {
    // begin communication ...
    Wire.begin();
    pinMode(intPin, INPUT);

    // verification ...
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) {
      if (calibrate) {
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
      }
      myIMU.initMPU9250();

      byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
      if (d == 0x48) {
        // Get magnetometer calibration from AK8963 ROM
        myIMU.initAK8963(myIMU.factoryMagCalibration);
      } // else fail

      // Get sensor resolutions, only need to do this once
      myIMU.getAres();
      myIMU.getGres();
      myIMU.getMres();

      if (calibrate) {
        myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
      }
    } // else fail


  }

  void read() {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

      // TODO : handle accel Bias?

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                           myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                           myIMU.mx, myIMU.mz, myIMU.deltat);
    qw = *getQ();
    qx = *(getQ() + 1);
    qy = *(getQ() + 2);
    qz = *(getQ() + 3);

    if (!ahrs)
    {
      myIMU.delt_t = millis() - myIMU.count;
      if (myIMU.delt_t > 500)
      {
        // acc (ax-ay-az) in G
        // gyro (gx-gy-gz) is in deg/sec
        // mag (mx-my-mz) in mG
        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        myIMU.count = millis();
      } // if (myIMU.delt_t > 500)
    } // if (!ahrs)
    else
    {
      // Serial print and/or display at 0.5 s rate independent of data rates
      myIMU.delt_t = millis() - myIMU.count;

      // update LCD once per half-second independent of read rate
      if (myIMU.delt_t > 500)
      {
        myIMU.count = millis();
        myIMU.sumCount = 0;
        myIMU.sum = 0;
      }

    }
  }
};
