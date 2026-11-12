String data_recv;

void serial_update() {
  while (1) {
    if (Serial.available()) {
      int16_t c = Serial.read();
      if (c!='\n' && c!=' ')
        data_recv += (char)c;
      else break;
    }
    else return;
  }
  String id = "";
  uint32_t val = 0;
  int i = 0;
  while (i<data_recv.length() && (data_recv[i]<48 || data_recv[i]>57)){
    id += data_recv[i];
    i++;
  }
  for (; i<data_recv.length(); i++){
    val = val*10+(data_recv[i]-48);
  }
  data_recv = "";
  set_data(val, id);
}
void set_data(uint32_t val, String id) {
  if (id == "s") {
    RPM = val;
    motor_RW.run(RPM);
  }
  else if (id == "ss") {
    RPM = -val;
    motor_RW.run(RPM);
  }
  else if (id == "br") {
    motor_RW.brake();
  }
  if (id == "k") {
    K1 = val/10000.0f;
  }
  else if (id == "kk") {
    K2 = val/10000.0f;
  }
  else if (id == "kkk") {
    K3 = val/10000.0f;
  }
  else if (id == "kkkk") {
    K4 = val/10000.0f;
  }
  else if (id == "a") {
    offset_angle = val/10.0f;
  }
  else if (id == "pos") {
    mx_pos = val;
  }
  else if (id == "p") {
    tuning = !(bool)tuning;
    reset();
  }
  else if (id == "d") {
    deadband = val;
  }
  else if (id == "t") {
    threshold = val;
  }
  else if (id == "ka") {
    sensor._k_accel = val/100.0f;
  }
  else if (id == "ta") {
    sensor._threshold_a = val/1.0f;
  }
  else if (id == "gl") {
    gyro_limit = val/10.0f;
  }
}

void telemetryTask() {
  uint32_t time = millis();
  if (time-t_telemetryTask >= 10){
    t_telemetryTask = time;
    serial_update();
    if (radioData.update()) {run(); }

    log(radioData[1]);
    // log(encoder.count);
    // log(pos_RW);
    // log(speed);
    // log(K1);
    // log(K2);
    // log(K3);
    // log(K4);
    Serial.println("");
  }
}