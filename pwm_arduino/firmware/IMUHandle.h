#include "quaternionFilters.h"
#include "MPU9250.h"

/*
    Adaptation of MPU9250 Basic Example Code by Kris Winer (04/01/2014)
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

  MPU9250 _device;

  bool _mag_cal;
  bool _gyro_cal;

  IMUHandle(const int intPin):
    intPin(intPin), calibrate(false), ahrs(false),
    _mag_cal(false), _gyro_cal(false)
  {
    // todo : handle calibration requests
  }

  void setup() {
    // begin communication ...
    Wire.begin();
    pinMode(intPin, INPUT);

    // verification ...
    byte c = _device.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) {
      if (true) {
        _device.calibrateMPU9250(_device.gyroBias, _device.accelBias);
      }
      _device.initMPU9250();

      byte d = _device.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
      if (d == 0x48) {
        // Get magnetometer calibration from AK8963 ROM
        _device.initAK8963(_device.factoryMagCalibration);
      } // else fail

      // Get sensor resolutions, only need to do this once
      _device.getAres();
      _device.getGres();
      _device.getMres();

      if (calibrate) {
        _device.magCalMPU9250(_device.magBias, _device.magScale);
      }
    } // else fail
  }

  void set_mag_bias(float x, float y, float z) {
    _device.magBias[0] = x;
    _device.magBias[1] = y;
    _device.magBias[2] = z;
  }
  void set_mag_scale(float x, float y, float z) {
    //_device.magScale[0] = x;
    //_device.magScale[1] = y;
    //_device.magScale[2] = z;
  }
  void gyro_cal() {
    _device.calibrateMPU9250(_device.gyroBias, _device.accelBias);
  }

  void mag_cal() {

  }

  bool read() {
    if (_mag_cal || _gyro_cal) {
      return false;
    }
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (_device.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      _device.readAccelData(_device.accelCount);  // Read the x/y/z adc values

      // TODO : handle accel Bias?

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      _device.ax = (float)_device.accelCount[0] * _device.aRes; // - _device.accelBias[0];
      _device.ay = (float)_device.accelCount[1] * _device.aRes; // - _device.accelBias[1];
      _device.az = (float)_device.accelCount[2] * _device.aRes; // - _device.accelBias[2];

      _device.readGyroData(_device.gyroCount);  // Read the x/y/z adc values

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      _device.gx = (float)_device.gyroCount[0] * _device.gRes;
      _device.gy = (float)_device.gyroCount[1] * _device.gRes;
      _device.gz = (float)_device.gyroCount[2] * _device.gRes;

      _device.readMagData(_device.magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      _device.mx = (float)_device.magCount[0] * _device.mRes
                   * _device.factoryMagCalibration[0] - _device.magBias[0];
      _device.my = (float)_device.magCount[1] * _device.mRes
                   * _device.factoryMagCalibration[1] - _device.magBias[1];
      _device.mz = (float)_device.magCount[2] * _device.mRes
                   * _device.factoryMagCalibration[2] - _device.magBias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    _device.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(_device.ax, _device.ay, _device.az, _device.gx * DEG_TO_RAD,
                           _device.gy * DEG_TO_RAD, _device.gz * DEG_TO_RAD, _device.my,
                           _device.mx, _device.mz, _device.deltat);
    qw = *getQ();
    qx = *(getQ() + 1);
    qy = *(getQ() + 2);
    qz = *(getQ() + 3);

    if (!ahrs)
    {
      _device.delt_t = millis() - _device.count;
      if (_device.delt_t > 500)
      {
        // acc (ax-ay-az) in G
        // gyro (gx-gy-gz) is in deg/sec
        // mag (mx-my-mz) in mG
        _device.tempCount = _device.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        _device.temperature = ((float) _device.tempCount) / 333.87 + 21.0;
        _device.count = millis();
      } // if (_device.delt_t > 500)
    } // if (!ahrs)
    else
    {
      // Serial print and/or display at 0.5 s rate independent of data rates
      _device.delt_t = millis() - _device.count;

      // update LCD once per half-second independent of read rate
      if (_device.delt_t > 500)
      {
        _device.count = millis();
        _device.sumCount = 0;
        _device.sum = 0;
      }

    }
    return true;
  }
};
