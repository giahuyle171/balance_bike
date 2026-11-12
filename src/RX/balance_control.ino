#define limit(x, a, b) ((x>b) ? b : ((x<a) ? a : x))

float K1 = 44;
float K2 = 0.5;
float K3 = 0.06;
float K4 = 0.0105;
// float Ka  = 0.01;

float gyroYfilt = 0;
float k_gyro = 0.4;
float alpha_pos = 1;
int32_t pos_RW = 0, mx_pos = 5000;
float speed = 0;
float pos = 0;

int16_t deadband = 30;
int16_t threshold = 10;
int16_t max_pwm = 255;

float gyro_limit = 3;
float angle_fix = 0.001; 

float offset_angle = 1.5;
int16_t pwm = 0;

uint32_t t_controller = 0;

void balance_control() {
  uint32_t time = millis();
  float dt = (time-t_controller)/10.0f;
  t_controller = time;

  speed = encoder.count/dt;
  pos_RW = limit(pos_RW + encoder.count, -mx_pos, mx_pos);
  encoder.count = 0;;

  gyroYfilt = gyroYfilt*(1-k_gyro) + k_gyro*aSensor.GyY*0.0152587;
  gyroYfilt = limit(gyroYfilt, -gyro_limit, gyro_limit);

  float error = aSensor.angle-offset_angle;
  pwm = error*K1 + gyroYfilt*K2 + speed*K3 + pos_RW*K4;
  pwm = limit(pwm, -max_pwm, max_pwm);

  if (abs(pwm) > threshold) {
    float k = (10.0-(float)min(10, abs(speed)))/10.0;
    int8_t dir = (pwm>0?1:-1);
    pwm += dir*k*deadband;
  }
  else pwm = 0;
  motor_RW.run(pwm);
}

void reset() {
  pos_RW = 0;
  pos = 0;
  gyroYfilt = 0;
  motor_RW.run(0);
  motor_RW.brake();
  encoder.reset();
}

