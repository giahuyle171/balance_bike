import serial
import time

SERIAL_PORT = 'COM8'
BAUD_RATE = 115200
TARGET_RPM = 300 
TEST_DURATION = 2.0

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(10)

def send_cmd(cmd):
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.01)

def evaluate_pid(kp, ki, kd):
    send_cmd(f"p{int(kp*1000)}")
    send_cmd(f"i{int(ki*1000)}")
    send_cmd(f"d{int(kd*1000)}")
    
    send_cmd("s0")
    time.sleep(1.0) 
    ser.flushInput()
    
    send_cmd(f"s{TARGET_RPM}")
    
    start_time = time.time()
    total_penalty = 0.0
    samples = 0
    
    while (time.time() - start_time) < TEST_DURATION:
        elapsed_time = time.time() - start_time
        
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8').strip()
                vals = line.split(',')
                if len(vals) >= 2:
                    current_rpm = float(vals[1])
                    error = TARGET_RPM - current_rpm
                    
                    if error > 0:
                        penalty = error * elapsed_time 
                    else:
                        penalty = abs(error) * 20.0 
                        
                    total_penalty += penalty
                    samples += 1
            except:
                pass
    return total_penalty / max(samples, 1)


print("Start (TWIDDLE)...")

p = [0.1, 0.0, 0.0]
p = [3.749023256493648, 0.27974983358324107, 0.09971313647581159]

dp = [0.1, 0.01, 0.01] 

best_err = evaluate_pid(p[0], p[1], p[2])
print(f"PID = {p}, Điểm phạt = {best_err:.2f}")

iteration = 0

try:
    while sum(dp) > 0.001:
        iteration += 1
        print(f"\n--- Vòng {iteration} --- Bước nhảy hiện tại: {dp}")
        
        for i in range(len(p)):
            p[i] += dp[i]
            err = evaluate_pid(p[0], p[1], p[2])
            
            if err < best_err:
                # Nếu tốt hơn: Giữ nguyên thay đổi, tăng bước nhảy để "đi tiếp"
                best_err = err
                dp[i] *= 1.1
                print(f"TỐT HƠN! Tăng tham số {i}. PID: {p} | Điểm phạt mới: {best_err:.2f}")
            else:
                # Nếu tệ đi: Thử trừ ngược lại
                p[i] -= 2 * dp[i]
                # Tránh để PID mang số âm
                if p[i] < 0: p[i] = 0.0
                
                err = evaluate_pid(p[0], p[1], p[2])
                
                if err < best_err:
                    # Nếu hướng giảm tốt hơn: Giữ nguyên, tăng bước nhảy
                    best_err = err
                    dp[i] *= 1.1
                    print(f"TỐT HƠN! Giảm tham số {i}. PID: {p} | Điểm phạt mới: {best_err:.2f}")
                else:
                    # Nếu cả tăng và giảm đều tệ: Quay về ban đầu, thu nhỏ bước nhảy lại
                    p[i] += dp[i]
                    dp[i] *= 0.9
except KeyboardInterrupt:
    pass
                
send_cmd("s0")
print(f"\n====================================")
print(f"HOÀN THÀNH! BỘ PID TỐT NHẤT LÀ:")
print(f"Kp = {p[0]:.4f}")
print(f"Ki = {p[1]:.4f}")
print(f"Kd = {p[2]:.4f}")
print(f"Điểm phạt nhỏ nhất: {best_err:.2f}")