import numpy as np
import pandas as pd
from scipy.optimize import differential_evolution
import static

class DerivativeWelfordVariance:
    def __init__(self, equivalent_samples=10):
        self.mean = 0.0
        self.variance = 0.0
        self.last_raw = 0.0
        self.alpha = 1.0 / equivalent_samples

    def update(self, raw_data):
        delta_val = raw_data - self.last_raw
        self.last_raw = raw_data

        error = delta_val - self.mean
        self.mean += self.alpha * error
        self.variance = (1.0 - self.alpha) * (self.variance + self.alpha * error * (delta_val - self.mean))
        return self.variance

class DualAdaptiveKalmanFilter:
    def __init__(self, r_min, est_e, q_min, threshold, window_size=10):
        self.r_min = r_min
        self.err_estimate = est_e
        self.q_min = q_min
        self.threshold = threshold
        self.window_size = window_size
        self.reset()

    def reset(self):
        self.current_estimate = 0.0
        self.last_estimate = 0.0
        self.err_estimate_var = self.err_estimate
        self.welford_R = DerivativeWelfordVariance(self.window_size)

    def run_filter(self, data_array, gamma_q, gamma_r):
        self.reset()
        filtered_output = []
        
        for mea in data_array:
            variance = self.welford_R.update(mea)
            adaptive_r = self.r_min + (variance * gamma_r)

            error = abs(mea - self.last_estimate)
            adaptive_q = self.q_min
            if error > self.threshold:
                excess_error = error - self.threshold
                adaptive_q += (excess_error * excess_error) * gamma_q

            self.err_estimate_var += adaptive_q
            kalman_gain = self.err_estimate_var / (self.err_estimate_var + adaptive_r)
            self.current_estimate = self.last_estimate + kalman_gain * (mea - self.last_estimate)
            self.err_estimate_var = (1.0 - kalman_gain) * self.err_estimate_var
            
            self.last_estimate = self.current_estimate
            filtered_output.append(self.current_estimate)
            
        return np.array(filtered_output)


FILE_PATH = "data.csv"
df = pd.read_csv(FILE_PATH)
raw_data = df['raw'].values

R_MIN, Q_MIN, THRESHOLD = static.analyze_static_parameters(FILE_PATH)
EST_E = 1.0

kf = DualAdaptiveKalmanFilter(R_MIN, EST_E, Q_MIN, THRESHOLD)

def cost_function(params):
    gamma_q, gamma_r = params
    
    filtered = kf.run_filter(raw_data, gamma_q, gamma_r)
    
    smoothness_loss = np.sum(np.diff(filtered) ** 2)

    tracking_loss = np.sum((raw_data - filtered) ** 2)

    total_loss = 0.8 * smoothness_loss + 0.2 * tracking_loss
    return total_loss

bounds = [(1e-7, 1e-1), (1e-4, 5.0)]

result = differential_evolution(cost_function, bounds, seed=42, maxiter=50)

best_gamma_q, best_gamma_r = result.x
print(f"-> r_min          : {R_MIN:.4f}")
print(f"-> q_min          : {Q_MIN:.4f}")
print(f"-> Gamma R        : {best_gamma_r:.7f}")
print(f"-> Gamma Q        : {best_gamma_q:.7f}")
print(f"-> threshold      : {THRESHOLD:.4f}")