template <typename T>
void log(T s) {
  Serial.print(s);
  Serial.print(" ");
}
int compare(const void *a, const void *b) {
  return (*(int*)a - *(int*)b);
};

void run() {
  if (radioData[4]==0) {
    if (radioData[5]) {
      motor_RW.run(RPM);
    }
    else motor_RW.run(radioData[0]);
  }
  motor_M.run(radioData[1]);
}

void controlTask() {
  uint32_t time = millis();
  if (time-t_controlTask >= 10) {
    t_controlTask = time;
    // control_trigger = false;

    aSensor.angle_calc();
    sensor.update();
    // if (!radioData[4]) return;
    if (tuning) balance_control();

  }
}

