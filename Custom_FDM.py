import time
import numpy as np
import pandas as pd
from flightgear_python.fg_if import FDMConnection

# Define 10s acceleration profile
timeline = np.array([0, 10])  # seconds
# Assume current airspeed ~10 m/s, target = 2x = 20 m/s
airspeed = np.array([0, 20])  # relative increase (we'll add to current)
rpm      = np.array([1000, 2200])  # engine RPM ramp

df = pd.DataFrame({
    "time": timeline,
    "airspeed": airspeed,
    "rpm": rpm
})

start_time = time.time()

def interpolate_target(elapsed):
    target_vcas = np.interp(elapsed, df["time"], df["airspeed"])
    target_rpm  = np.interp(elapsed, df["time"], df["rpm"])
    return target_vcas, target_rpm

def fdm_callback(fdm_data, event_pipe):
    elapsed = time.time() - start_time
    if elapsed > 10: elapsed = 10  # clamp at 10s

    target_vcas, target_rpm = interpolate_target(elapsed)

    # Control inputs: gently move toward targets
    fdm_data.rpm[0] += 0.01 * (target_rpm - fdm_data.rpm[0])
    fdm_data.elevator = 0.0  # keep nose level
    fdm_data.left_aileron = 0.0
    fdm_data.right_aileron = 0.0
    fdm_data.rudder = 0.0

    # --- Print logs ---
    print(f"[{elapsed:.1f}s] TargetV={target_vcas:.2f} | TargetRPM={target_rpm:.1f} || "
          f"Airspeed={fdm_data.vcas:.2f} | Alt={fdm_data.alt_m:.2f} | "
          f"Pitch={fdm_data.theta_rad:.3f} | Roll={fdm_data.phi_rad:.3f}")

    return fdm_data

if __name__ == '__main__':
    fdm_conn = FDMConnection()
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)
    fdm_conn.start()

    while True:
        time.sleep(0.05)
