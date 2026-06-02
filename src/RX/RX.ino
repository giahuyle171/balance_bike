#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
// #include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
// #include <Simplefilter.h>

void log(float s) {
  Serial.print(s);
  Serial.print(" ");
}
void log(int s) {
  Serial.print(s);
  Serial.print(" ");
}
void log(unsigned int s) {
  Serial.print(s);
  Serial.print(" ");
}
int compare(const void *a, const void *b) {
  return (*(int*)a - *(int*)b);
};
class Motor {
  public:
    uint8_t _m1,_m2;
    int16_t pwm = 0;
    Motor(uint8_t m1, uint8_t m2) {
      _m1 = m1;
      _m2 = m2;
      pinMode(_m1, OUTPUT);
      pinMode(_m2, OUTPUT);
    }
    void run(int16_t val) {
      pwm = val;
      if (pwm>0){
        analogWrite(_m1, min(pwm, 255));
        analogWrite(_m2, 0);
      }
      else if (pwm<0){
        analogWrite(_m1, 0);
        analogWrite(_m2, min(-pwm, 255));
      }
      else {
        digitalWrite(_m1, 0);
        digitalWrite(_m2, 0);
      }
    }
    void brake() {
      digitalWrite(_m1, 1);
      digitalWrite(_m2, 1);
    }
};
// class Motor {
//   private:
//     volatile uint8_t* _ocr1;   
//     volatile uint8_t* _ocr2;   
//     volatile uint8_t* _port1;  
//     volatile uint8_t* _port2;  
//     volatile uint8_t* _tccr1;  
//     volatile uint8_t* _tccr2;  
    
//     uint8_t _bit1, _bit2;      
//     uint8_t _com1, _com2;      

//     void setupPin(uint8_t pin, volatile uint8_t** ocr, volatile uint8_t** port, uint8_t* bit, volatile uint8_t** tccr, uint8_t* com) {
//         switch(pin) {
//             case 3:  *ocr = &OCR2B;   *port = &PORTD; *bit = PD3; *tccr = &TCCR2A;   *com = (1<<COM2B1); DDRD |= (1<<DDD3); break;
//             case 5:  *ocr = &OCR0B;   *port = &PORTD; *bit = PD5; *tccr = &TCCR0A;   *com = (1<<COM0B1); DDRD |= (1<<DDD5); break;
//             case 6:  *ocr = &OCR0A;   *port = &PORTD; *bit = PD6; *tccr = &TCCR0A;   *com = (1<<COM0A1); DDRD |= (1<<DDD6); break;
            
//             // CHÂN 9: Chân thuần Digital (Không đụng vào Timer 1 để giữ cấu hình ngắt)
//             case 9:  
//                 *ocr = nullptr;       
//                 *port = &PORTB;       // Chân 9 thuộc PORTB
//                 *bit = PB1;           // Chân 9 là Bit số 1
//                 *tccr = nullptr;      
//                 *com = 0; 
//                 DDRB |= (1<<DDB1);    // Cấu hình OUTPUT trực tiếp bằng thanh ghi
//                 break;
//         }
//     }

//     inline void writePWM(volatile uint8_t* ocr, volatile uint8_t* port, uint8_t bit, volatile uint8_t* tccr, uint8_t com, uint8_t val) {
//         if (tccr == nullptr) { 
//             // Xử lý cho chân Digital thuần túy (Chân 9)
//             if (val == 0) *port &= ~(1 << bit); // LOW
//             else          *port |= (1 << bit);  // HIGH
//         } else {
//             // Xử lý cho các chân băm xung PWM cứng (Chân 3, 5, 6)
//             if (val == 0) {
//                 *tccr &= ~com;        
//                 *port &= ~(1 << bit); 
//             } else if (val >= 255) {
//                 *tccr &= ~com;        
//                 *port |= (1 << bit);  
//             } else {
//                 *tccr |= com;         
//                 *ocr = val;           
//             }
//         }
//     }

//   public:
//     int16_t pwm = 0;

//     Motor(uint8_t m1_pin, uint8_t m2_pin) {
//         setupPin(m1_pin, &_ocr1, &_port1, &_bit1, &_tccr1, &_com1);
//         setupPin(m2_pin, &_ocr2, &_port2, &_bit2, &_tccr2, &_com2);
//     }

//     void run(int16_t val) {
//         pwm = val;
//         if (pwm > 0) {
//             if (pwm > 255) pwm = 255;
//             writePWM(_ocr1, _port1, _bit1, _tccr1, _com1, pwm);
//             writePWM(_ocr2, _port2, _bit2, _tccr2, _com2, 0);
//         } 
//         else if (pwm < 0) {
//             if (pwm < -255) pwm = -255;
//             writePWM(_ocr1, _port1, _bit1, _tccr1, _com1, 0);
//             writePWM(_ocr2, _port2, _bit2, _tccr2, _com2, -pwm); // Chân 9 nhận giá trị dương sẽ tự nhảy lên HIGH
//         } 
//         else {
//             writePWM(_ocr1, _port1, _bit1, _tccr1, _com1, 0);
//             writePWM(_ocr2, _port2, _bit2, _tccr2, _com2, 0);
//         }
//     }

//     void brake() {
//         if (_tccr1 != nullptr) *_tccr1 &= ~_com1;  *_port1 |= (1 << _bit1);
//         if (_tccr2 != nullptr) *_tccr2 &= ~_com2;  *_port2 |= (1 << _bit2);
//     }
// };
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
    const float RPM_MAX = 850;
    const float VCC_MAX = 16.5;
    const float VOUT_MAX = (VCC_MAX*3300/(10000+3300))/5*1023;
    const float backEMF_MAX = VOUT_MAX - 30;
    const float _k_emf = RPM_MAX/backEMF_MAX;
    const float _delay_time = 500;
    uint8_t _pin_m1, _pin_m2, _pin_vcc;
    Motor& _motor;
  public:
    DAKFilter filter;
    float _back_emf, _raw_rpm, _rpm, _vcc;
    Sensor(uint8_t m1, uint8_t m2, uint8_t vcc, Motor& m)
      :filter(31.96, 1.4, 0.0071, 0.001, 15.03), _motor(m)
    // :filter(33.54, 1.6769, 0.0671, 0.00698, 16.7858)
    {
      _pin_m1 = m1;
      _pin_m2 = m2;
      _pin_vcc = vcc;
    }

    float readMedian(int pin, int n) {
      int data[n];
      data[0] = analogRead(pin);
      if (data[0] < 10) return 0;
      for (int i = 1; i<n; i++) data[i] = analogRead(pin);
      
      qsort(data, n, sizeof(int), compare);
      return data[int(n*0.5f)];
    }

    void readBackEMF() {
      float prev_pwm = _motor.pwm;
      _motor.run(0);
      delayMicroseconds(_delay_time);

      float v1 = readMedian(_pin_m1, 5);
      float v2 = readMedian(_pin_m2, 5);

      _motor.run(prev_pwm);
      _back_emf = v2-v1;
    }
    void update() {
      readBackEMF();
      _raw_rpm = _back_emf * _k_emf;
      if (_raw_rpm < 5 && _raw_rpm >-5) _raw_rpm = 0;

      _rpm = filter.updateEstimate(_raw_rpm);
      if (_rpm < 5 && _rpm >-5) _rpm = 0;
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

class AngleSensor {
  private:
    float _deadband_gyro;
  public:
    MPU6050 mpu;
    float angle, gyro;
    AngleSensor(float deadband_gyro)
      : mpu(Wire), _deadband_gyro(deadband_gyro) {}
    void init(){
      Wire.begin();
      Wire.beginTransmission(0x68); 
      Wire.write(0x1A);
      Wire.write(0x06);
      Wire.endTransmission();
      mpu.begin();
      mpu.calcGyroOffsets(true);
      update();
    }
    void update() {
      mpu.update();
      angle = mpu.getAngleY();
      gyro = mpu.getGyroY();
      if (gyro < _deadband_gyro & gyro > -_deadband_gyro) gyro = 0;
    }
};

class PID {
  public:
    float _kp, _ki, _kd;
    float _prev_input = 0;
    float _setpoint = 0;
    float _out_max, _int_max;
    float error, integral = 0, output;
    PID(float kp=0, float ki=0, float kd=0, float out_max=1e9, float int_max=1e9) {
      _kp = kp;
      _ki = ki;
      _kd = kd;
      _out_max = out_max;
      _int_max = int_max;
    }
    float computePID(float input) {
      error = _setpoint - input; 
      output = p() + i() + d(input);
      _prev_input = input;
      if (output>_out_max) output = _out_max;
      else if (output < -_out_max) output = -_out_max;
      return output;
    }
    float computePcD(float input, float d) {
      error = _setpoint - input; 
      output = p() + custom_d(d);
      if (output>_out_max) output = _out_max;
      else if (output < -_out_max) output = -_out_max;
      return output;
    }
    inline float p() { return error*_kp; }
    inline float i() {
      integral += error;
      if (integral>_int_max) integral = _int_max;
      else if (integral < -_int_max) integral = -_int_max;
      return integral*_ki;
    }
    inline float d(float input) { return -(input-_prev_input)*_kd; }
    inline float custom_d(float d) { return d*_kd; }
    inline void setpoint(float setpoint) { _setpoint = setpoint; }
    inline void reset() { integral = 0.0f; _prev_input = 0.0f; }
};

class BalanceControler {
  public:
    PID& sPID;
    PID& aPID;
    float k_dr;
    int16_t _rpm_max;
    int16_t _deadband;

    BalanceControler(PID& sPID, PID& aPID, int16_t rpm_max, int16_t deadband)
      : sPID(sPID), aPID(aPID), _rpm_max(rpm_max), _deadband(deadband)
    {}
    void init(float setpoint) {
      aPID.setpoint(setpoint);
    }
    int16_t update(float angle, float gyro, float rpm) {
      aPID.computePcD(angle, gyro);
      sPID._setpoint += aPID.output;

      if (sPID._setpoint>_rpm_max) sPID._setpoint = _rpm_max;
      else if (sPID._setpoint < -_rpm_max) sPID._setpoint = -_rpm_max;

      int16_t pwm = sPID.computePID(rpm);
      if (pwm<-0) pwm -= _deadband;
      else if (pwm>0) pwm += _deadband;
      else pwm = 0;
      return pwm;
    }
};

void setupTimer1() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // Prescaler = 64; ticks = 16MHz/(64*100Hz) = 2500;
  OCR1A = 2499;
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<CS11) | (1<<CS10);
  TIMSK1 |= (1<< OCIE1A);
  sei();
}
volatile bool control_trigger = false;

Motor motor_RW(5,3);
Motor motor_M(6,9);

RadioData radioData(8,10);

Sensor sensor(A6, A7, A0, motor_RW);
AngleSensor aSensor(0.3);

PID speedPID(3.7490,0.2797,0.1002, 205, 410);
PID anglePID(0,0,0,500,500);

BalanceControler bControler(speedPID, anglePID, 800, 50);

int16_t RPM = 0;


void run() {
  if (radioData[4]==0) {
    if (radioData[5]) {
      motor_RW.run(RPM);
    }
    else motor_RW.run(radioData[0]);
  }
  motor_M.run(radioData[1]);
}

class SerialData {
  public:
    String data_recv = "";
    void update() {
      if (Serial.available()) {
        data_recv = "";
        for (int c = Serial.read(); c!='\n' && c!=' '; c = Serial.read())
          if (c != -1) data_recv += char(c);
        set(data_recv);
      }
    }
    void set(String s) {
      String id = "";
      uint64_t val = 0;
      int i = 0;
      while (s[i] < 48 || s[i] > 57){
        id += s[i];
        i++;
      }
      for (; i<s.length(); i++){
        val = val*10+(s[i]-48);
      }
      if (id == "s") {
        RPM = val;
      }
      else if (id == "ss") {
        RPM = -val;
      }  
      else if (id == "p") {
        anglePID._kp = val/1000.0f;
      }  
      else if (id == "i") {
        anglePID._ki = val/1000.0f;
      }
      else if (id == "d") {
        anglePID._kd = val/1000.0f;
      }
    }
};

SerialData sData;

void controlTask() {
  if (control_trigger) {
    control_trigger = false;
    aSensor.update();
    sensor.update();
    if (!radioData[4]) return;
    int16_t pwm = bControler.update(aSensor.angle,aSensor.gyro,sensor._rpm);
    motor_RW.run(pwm);
    // motor_RW.run(RPM);

  }
}

float t_telemetryTask = 0;
void telemetryTask() {
  float time = millis();
  if (time-t_telemetryTask >= 10){
    t_telemetryTask = time;

    if (radioData.update()) {run(); }
    sData.update();

    log(-700);
    log(700);
    log(sensor._raw_rpm);
    log(sensor._rpm);
    log(aSensor.angle);
    log(aSensor.gyro);
    Serial.println("");

    // Serial.print(time); Serial.print(",");
    // Serial.println(sensor._rpm);
  }
}

int main(void) {
  init();
  Serial.begin(115200);
  setupTimer1();

  radioData.init();
  radioData.openReadingPipe(1, (const byte*)"12345");
  radioData.setPALevel(RF24_PA_MIN);
  radioData.setChannel(80);
  radioData.setDataRate(RF24_250KBPS);  
  radioData.startListening();

  aSensor.init();

  // bControler.init(aSensor.angle);
  bControler.init(-86);

  while (1) {
    controlTask();
    telemetryTask();
  }

  return 0;
}
ISR(TIMER1_COMPA_vect) {
  control_trigger = true;
}