import serial
import time
import csv

FILE_PATH = 'data2.csv'
SERIAL_PORT = 'COM8' 
BAUD_RATE = 115200
STEP_PWM = 200      
DURATION = 4.0    

print(f"Connect {SERIAL_PORT}...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(10) 

ser.flushInput()

print("Start")
start_time = time.time()
data = []

while (time.time() - start_time) < 0.5:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if "," in line:
            data.append(line + ",0")

print(f"PWM = {STEP_PWM}")
ser.write(f"s{STEP_PWM}\n".encode())

while (time.time() - start_time) < DURATION:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if "," in line:
            data.append(line + f",{STEP_PWM}")

ser.write("s0\n".encode())
ser.close()

with open(FILE_PATH, 'w', newline='') as f:
    f.write("Time,RPM,PWM\n")
    for row in data:
        f.write(row + "\n")

print("Done")
