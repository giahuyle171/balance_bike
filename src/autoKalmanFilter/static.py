import pandas as pd
import numpy as np

def analyze_static_parameters(csv_file):
    print("analyzing...\n")
    
    df = pd.read_csv(csv_file)
    raw = df['raw'].values
    
    deltas = np.abs(np.diff(raw))
    
    rolling_var = df['raw'].rolling(window=20).var().dropna().values

    threshold_percentile = np.percentile(rolling_var, 10)
    static_indices = np.where(rolling_var <= threshold_percentile)[0]
    
    if len(static_indices) == 0:
        print("error noise")
        return

    static_variances = rolling_var[static_indices]
    
    valid_delta_indices = [i for i in static_indices if i < len(deltas)]
    static_deltas = deltas[valid_delta_indices]

    r_min = np.mean(static_variances) * 1.5
    
    threshold = np.percentile(static_deltas, 99) + 1.0
    
    q_min = r_min * 0.05
    return r_min, q_min, threshold
    

FILE_PATH = "data.csv"
analyze_static_parameters(FILE_PATH)