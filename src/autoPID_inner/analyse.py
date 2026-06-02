import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# 1. ĐỌC DỮ LIỆU
df = pd.read_csv('data2.csv')
t = df['Time'].values / 1000.0 # Chuyển ms sang giây
y = df['RPM'].values
u = df['PWM'].values

# Đưa thời gian về bắt đầu từ 0
t = t - t[0]

# Xác định thời điểm Step Input xảy ra và mức PWM thay đổi
step_index = np.where(u > 0)[0][0]
t_step = t[step_index]
delta_u = u[step_index] - u[0]

# 2. ĐỊNH NGHĨA MÔ HÌNH TOÁN HỌC FOPDT
# Hàm mô phỏng tốc độ RPM theo thời gian
def fopdt(t, K, T, L):
    y_sim = np.zeros_like(t)
    for i, t_i in enumerate(t):
        if t_i > (t_step + L):
            y_sim[i] = K * delta_u * (1 - np.exp(-(t_i - t_step - L) / T))
    return y_sim

# 3. KHỚP DỮ LIỆU (CURVE FITTING)
# Dự đoán ban đầu: Gain (K)=2, Time const (T)=0.2s, Dead time (L)=0.05s
p0 = [2.0, 0.2, 0.05]
# Giới hạn thông số (K, T, L đều phải > 0)
bounds = ([0.01, 0.01, 0.001], [10.0, 5.0, 1.0])

try:
    popt, _ = curve_fit(fopdt, t, y, p0=p0, bounds=bounds)
    K, T, L = popt
    print("--- KẾT QUẢ NHẬN DẠNG HỆ THỐNG ---")
    print(f"Độ lợi (K): {K:.4f} RPM/PWM")
    print(f"Hằng số thời gian (T): {T:.4f} giây")
    print(f"Độ trễ (L): {L:.4f} giây\n")
except Exception as e:
    print("Không thể khớp mô hình. Dữ liệu có thể bị nhiễu quá mạnh.", e)
    exit()

# 4. TÍNH TOÁN PID (Theo Ziegler-Nichols vòng hở)
# Lưu ý: Code Arduino của bạn dùng chu kỳ ngắt dt = 10ms (0.01s)
# Do biến integral = integral + error, nên Ki(code) = Ki(thực) * dt
# Do biến derivative = input - prev_input, nên Kd(code) = Kd(thực) / dt
dt = 0.01 

Kp_continuous = (1.2 * T) / (K * L)
Ti = 2.0 * L
Td = 0.5 * L

Ki_continuous = Kp_continuous / Ti
Kd_continuous = Kp_continuous * Td

# Quy đổi sang PID rời rạc cho đúng với Class PID của bạn
Kp_code = Kp_continuous
Ki_code = Ki_continuous * dt
Kd_code = Kd_continuous / dt

print("--- THÔNG SỐ PID CHO CODE ARDUINO CỦA BẠN ---")
print(f"Kp = {Kp_code:.4f}  (Gõ vào Serial: p{int(Kp_code*1000)})")
print(f"Ki = {Ki_code:.4f}  (Gõ vào Serial: i{int(Ki_code*1000)})")
print(f"Kd = {Kd_code:.4f}  (Gõ vào Serial: d{int(Kd_code*1000)})")

# 5. VẼ ĐỒ THỊ KIỂM CHỨNG
y_fit = fopdt(t, K, T, L)
plt.figure(figsize=(10, 5))
plt.plot(t, y, label='Dữ liệu thực tế (Đo được)', color='blue', alpha=0.6)
plt.plot(t, y_fit, label='Mô hình FOPDT (Khớp)', color='red', linestyle='--')
plt.axvline(x=t_step, color='green', linestyle=':', label='Kích hoạt PWM')
plt.title('System Identification: Khớp mô hình động cơ DC')
plt.xlabel('Thời gian (giây)')
plt.ylabel('Tốc độ (RPM)')
plt.legend()
plt.grid(True)
plt.show()