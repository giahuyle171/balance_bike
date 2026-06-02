import serial
import pandas as pd
import time

COM_PORT = 'COM8' 
BAUD_RATE = 9600
FILE_PATH = "data.csv"

data_list = []

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Connected {COM_PORT}")
    
    time.sleep(10)

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    val = float(line)
                    data_list.append(val)
                    print(f"-> {val}")
                except:
                    pass

except KeyboardInterrupt:
    print("\nDONE")
except Exception as e:
    print(f"error Serial: {e}")

finally:
    ser.close()
    if len(data_list) > 0:
        df = pd.DataFrame({'raw': data_list})
        df.to_csv(FILE_PATH, index=False)
        print(f"write {len(data_list)} to '{FILE_PATH}'.")