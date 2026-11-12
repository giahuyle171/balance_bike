#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "bmi160_sensor.h"

class Motor {
  public:
    uint8_t _m1,_m2;
    int16_t pwm = 0;
    bool _brake = 0;
    Motor(uint8_t m1, uint8_t m2) {
      _m1 = m1;
      _m2 = m2;
      pinMode(_m1, OUTPUT);
      pinMode(_m2, OUTPUT);
    }
    void run(int16_t val) {
      _brake = 0;
      pwm = val;
      if (pwm>0){
        analogWrite(_m1, min(pwm, 255));
        digitalWrite(_m2, 0);
      }
      else if (pwm<0){
        digitalWrite(_m1, 0);
        analogWrite(_m2, min(-pwm, 255));
      }
      else {
        digitalWrite(_m1, 0);
        digitalWrite(_m2, 0);
      }
    }
    void brake() {
      _brake = 1;
      digitalWrite(_m1, 1);
      digitalWrite(_m2, 1);
    }
};
class DerivativeWelfordVariance {
  public:
    float _mean, _variance, _alpha;
    float _last_raw;
    DerivativeWelfordVariance(int n = 10) {
      _mean = 0.0f;
      _variance = 0.0f;
      _last_raw = 0.0f;
      _alpha = 1.0f / n;
    }

    float update(float raw_data) {
      float delta_val = raw_data - _last_raw;
      _last_raw = raw_data;

      float error = delta_val - _mean;
      _mean += _alpha * error;
      _variance = (1.0f-_alpha)*(_variance+_alpha*error*(delta_val-_mean));
      return _variance; 
    }
    void setSamples(int n) {
      _mean = 0.0f;
      _variance = 0.0f;
      _last_raw = 0.0f;
      _alpha = 1.0f / n;
    }
};
class DAKFilter {
  private:    
    float _current_estimate, _last_estimate, _kalman_gain, _err_estimate;
  public:
    // ContinuousWelfordVariance _welford_R;
    DerivativeWelfordVariance _welford_R;
    float _gamma_q, _gamma_r;
    float _q_min, _r_min;
    float _threshold;
    float error;
    DAKFilter(float r_min, float q_min, float gamma_r, float gamma_q, float threshold, float est_e = 1, int window_size = 10) 
      : _welford_R(window_size)
    {
      _r_min = r_min;
      _q_min = q_min;
      _gamma_q = gamma_q;
      _gamma_r = gamma_r;
      _threshold = threshold;
      _err_estimate = est_e;
      _current_estimate = 0;
      _last_estimate = 0;
    }

    float updateEstimate(float mea) {
      float variance = _welford_R.update(mea);
      float adaptive_r = _r_min + (variance * _gamma_r);

      error = abs(mea - _last_estimate);
      // float adaptive_q = _q_min + (error * _gamma_q);
      float adaptive_q = _q_min;

      if (error > _threshold) {
        float excess = error-_threshold;
        adaptive_q += excess * excess * _gamma_q;
      }

      _err_estimate = _err_estimate + adaptive_q;
      _kalman_gain = _err_estimate / (_err_estimate + adaptive_r);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate = (1.0f - _kalman_gain) * _err_estimate;
      
      _last_estimate = _current_estimate;

      return _current_estimate;
    }
    void setGamma_q(float gamma) { _gamma_q = gamma; }
    void setGamma_r(float gamma) { _gamma_r = gamma; }
    void setRmin(float r) { _r_min = r; }
    void setQmin(float q) { _q_min = q; }
    void setThreshold(float t) {_threshold = t;}
};
class Sensor {
  private:
    const float _k_emf = 1.0;
    const float _k_offset = 1;
    const uint8_t _delay_time = 500;
    uint8_t _pin_m1, _pin_m2, _pin_vcc;
    Motor& _motor;
    int16_t _prev_bEmf = 0;
    uint32_t _prev_time = 0;
  public:
    float _k_accel = 0.3;
    float _threshold_a = 5;
    DAKFilter filter;
    float _back_emf, _raw_bEmf, _bEmf, _vcc, _accel;
    Sensor(uint8_t m1, uint8_t m2, uint8_t vcc, Motor& m)
      :filter(31.96, 1.4, 0.0071, 0.001, 15.03), _motor(m)
    // :filter(33.54, 1.6769, 0.0671, 0.00698, 16.7858)
    {
      _pin_m1 = m1;
      _pin_m2 = m2;
      _pin_vcc = vcc;
    }

    int16_t readMedian(int16_t a, int16_t b, int16_t c) {
      if ((a<=b && b<=c) || (c<=b && b<=a)) return b;
      if ((b<=a && a<=c) || (c<=a && a<=b)) return a;
      return c;
    }

    void readBackEMF() {
      float prev_pwm = _motor.pwm;
      bool brake = _motor._brake;
      _motor.run(0);
      delayMicroseconds(_delay_time);
      int16_t a1 = analogRead(_pin_m1); int16_t b1 = analogRead(_pin_m2);
      int16_t b2 = analogRead(_pin_m2); int16_t a2 = analogRead(_pin_m1);
      int16_t a3 = analogRead(_pin_m1); int16_t b3 = analogRead(_pin_m2);
      if (brake) _motor.brake();
      else _motor.run(prev_pwm);
      int16_t v1 = readMedian(a1, a2, a3);
      int16_t v2 = readMedian(b1, b2, b3);
      _back_emf = (float)(v2-v1);
    }
    void readVcc() {
      _vcc = analogRead(_pin_vcc);
    }
    void update() {
      readBackEMF();
      readVcc();
      _raw_bEmf = _back_emf * _k_emf;
      if (_raw_bEmf < 0) _raw_bEmf *= _k_offset;
      if (_raw_bEmf < 5 && _raw_bEmf >-5) _raw_bEmf = 0;

      _bEmf = filter.updateEstimate(_raw_bEmf);
      if (_bEmf < 5 && _bEmf >-5) _bEmf = 0;

      // uint32_t time = micros();
      // float raw_accel = (_bEmf-_prev_bEmf)*10000.0f/(time-_prev_time);
      // if (abs(raw_accel) < _threshold_a) 
      //   _accel = _accel*(1-_k_accel) + raw_accel*_k_accel;
      // else
      //   _accel = raw_accel;
      // _prev_time = time;
      _prev_bEmf = _bEmf;
    }
};
class RadioData: public RF24 {
  public:
    int16_t data[6] = {0,0,0,0,0,0};
    // joyleft, joyright, val1, val2, switch1, switch2];
    RadioData(int ce_pin, int csn_pin)
      :RF24(ce_pin, csn_pin) {}
    
    void init() {
      if (!RF24::begin()) {
        Serial.println("Error NRF24L01");
        while(1) {}
      }
    }
    bool update() {
      if (RF24::available()) {
        RF24::read(&data, sizeof(data));
        return 1;
      }
      return 0;
    }
    int& operator[](int i) {return data[i]; }
};

class Encoder {
  public:
    uint8_t encA, encB;
    int32_t count = 0;
    Encoder(uint8_t a, uint8_t b) { encA = a; encB = b;}
    void init() {
      // 4x
      pinMode(encA, INPUT_PULLUP);
      pinMode(encB, INPUT_PULLUP);
    }
    void update() {
       static int state = 0;
        state = (state << 2 | (digitalRead(encA) << 1) | digitalRead(encB)) & 0x0f;
        if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
          count--;
        } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
          count++;
        }
    }
    void getVelocity() {
      static int32_t prev_count = 0;
      float speed = count-prev_count;
      prev_count = count;
      return speed;
    }
    void reset() { count = 0; }
};
// volatile bool control_trigger = false;
Encoder encoder(2,3);

volatile bool tuning = false;
volatile uint32_t t_controlTask = 0;
volatile uint32_t t_telemetryTask = 0;

Motor motor_RW(10,9);
Motor motor_M(5,6);

RadioData radioData(8,7);

Sensor sensor(A6, A7, A0, motor_RW);
BMI160 aSensor;

int16_t RPM = 0;

int main(void) {
  init();
  Serial.begin(9600);
  // setupTimer1();

  // ADCSRA = (ADCSRA & 0xF8) | 0x05;
  ADCSRA = (ADCSRA & 0xF8) | 0x06; // Prescaler ADC = 64

  // TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  // TCCR2B = _BV(CS22);

  radioData.init();
  radioData.openReadingPipe(1, (const uint8_t*)"12345");
  radioData.setPALevel(RF24_PA_MAX);
  radioData.setChannel(90);
  radioData.setDataRate(RF24_250KBPS);  
  radioData.setAutoAck(false);
  radioData.startListening();

  // aSensor.init();

  // encoder.init();
  // attachInterrupt(digitalPinToInterrupt(encoder.encA), []() {encoder.update(); }, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoder.encB), []() {encoder.update(); }, CHANGE);

  while (1) {
    // controlTask();
    telemetryTask();
  }

  return 0;
}