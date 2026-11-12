#include <Wire.h>
#define IMU_ADDR 0x68
#define ACC_CONF 0x40
#define ACC_RANGE 0x41
#define GYR_CONF 0x42
#define GYR_RANGE 0x43

#define GYRO_XOUT_L 0x0C
#define GYRO_XOUT_H 0x0D
#define GYRO_YOUT_L 0x0E
#define GYRO_YOUT_H 0x0F
#define GYRO_ZOUT_L 0x10
#define GYRO_ZOUT_H 0x11

#define ACCEL_XOUT_L 0x12
#define ACCEL_XOUT_H 0x13
#define ACCEL_YOUT_L 0x14
#define ACCEL_YOUT_H 0x15
#define ACCEL_ZOUT_L 0x16
#define ACCEL_ZOUT_H 0x17

#define SENSORTIME_0 0x18
#define SENSORTIME_1 0x19
#define SENSORTIME_2 0x1A

#define CMD 0x7E

#define accSens 0x03    //  2g
#define gyroSens 0x02  // 500rad/s

class BMI160 {
  private:
    float _deadband_gyro;
    uint32_t _prev_time;
    uint32_t _dt;
  public:
    float AcX, AcY, AcZ, GyX, GyY, GyZ;
    float GyX_offset = 0;
    float angle = 0;
    float _k = 0.5;

    void init(){
      Wire.begin();
      delay(100);
      writeTo(IMU_ADDR, CMD, 0x11);
      delay(100);
      writeTo(IMU_ADDR, CMD, 0x15);
      delay(100);

      writeTo(IMU_ADDR, ACC_RANGE, accSens);
      writeTo(IMU_ADDR, GYR_RANGE, gyroSens);
      writeTo(IMU_ADDR, ACC_CONF, 0x08); // ODR 100HZ, ORS4: 0.11*ODR
      writeTo(IMU_ADDR, GYR_CONF, 0x08);
      delay(100);

      Serial.println();
      Serial.println();
      Serial.println("Offset...");

      float sum = 0;
      GyX_offset = 0;
      for (int i = 0; i<2048; i++) {
        update();
        sum += GyX;
        delay(1);
      }
      GyX_offset = sum / 2048;

      Serial.print("GyX offset: ");  Serial.println(GyX_offset);
      Serial.println();
      delay(1000);
    }
    void writeTo(int8_t device, int8_t address, int8_t value) {
      Wire.beginTransmission(device);
      Wire.write(address);
      Wire.write(value);
      Wire.endTransmission();
    }
    void request(int8_t device, int8_t address, int8_t size) {
      Wire.beginTransmission(device);
      Wire.write(address);
      Wire.endTransmission(false);
      Wire.requestFrom(device, size, true);
    }
    void update() {
      request(IMU_ADDR, GYRO_XOUT_L, 15);
      uint8_t buf[16] = {0};
      Wire.readBytes((char*)buf, 15);
      GyX = *(int16_t*)&buf[0];
      AcY = *(int16_t*)&buf[8];
      AcZ = *(int16_t*)&buf[10];
      // memcpy(&GyX, &buf[2], 2);
      // memcpy(&AcX, &buf[6], 2);
      // memcpy(&AcZ, &buf[10], 2);

      uint32_t time = *(uint32_t*)&buf[12];
      // uint32_t time = 0;
      // memcpy(&time, &buf[12], 4);

      _dt = (time - _prev_time) & 0x00FFFFFF;
      _prev_time = time;

      GyX -= GyX_offset;
    }
    void angle_calc() {
      update();
      angle += GyX*_dt/1680410; 
      float accel_angle = atan2(-AcY, -AcZ) * 57.29578;
      angle = angle*_k + accel_angle*(1-_k);
    }
};