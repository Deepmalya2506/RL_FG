import time
import math
import logging
from logging.handlers import RotatingFileHandler
from flightgear_python.fg_if import FDMConnection

# --- LOGGER SETUP ---
logger = logging.getLogger("FlightDynamicsLogger")
logger.setLevel(logging.INFO)
# Rotating log file: max 5MB, keep 5 backups
handler = RotatingFileHandler("flight_log.log", maxBytes=5*1024*1024, backupCount=5)
formatter = logging.Formatter('%(asctime)s | %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# --- SIMULATION CONSTANTS ---
ACCEL_K = 2.5        # Uniform acceleration (m/s^2) - selected for a smooth 10s roll
EARTH_RADIUS = 6378137.0 
FPS = 30
DT = 1.0 / FPS

class AircraftState:
    def __init__(self):
        self.velocity_mps = 0.0
        self.distance_m = 0.0
        self.start_time = None
        self.initial_lat = None
        self.initial_lon = None
        self.heading_rad = None  # To capture the runway heading

state = AircraftState()

def fdm_callback(fdm_data, event_pipe):
    curr_time = time.time()
    
    # Initialization on first packet
    if state.start_time is None:
        state.start_time = curr_time
        state.initial_lat = fdm_data['lat_rad']
        state.initial_lon = fdm_data['lon_rad']
        # Capture the direction the plane is facing so we move "Forward"
        state.heading_rad = fdm_data['psi_rad'] 
        logger.info(f"Simulation Started. Heading: {math.degrees(state.heading_rad):.2f} degrees")

    elapsed = curr_time - state.start_time

    # 1. Uniform Acceleration Logic (for 10 seconds)
    if elapsed <= 10.0:
        state.velocity_mps += ACCEL_K * DT
        state.distance_m += state.velocity_mps * DT
    else:
        # Stop accelerating after 10s, maintain speed
        pass

    # 2. Update Position based on Heading (Trigonometry)
    # delta_lat = distance * cos(heading)
    # delta_lon = distance * sin(heading) / cos(lat)
    lat_change = (state.distance_m * math.cos(state.heading_rad)) / EARTH_RADIUS
    lon_change = (state.distance_m * math.sin(state.heading_rad)) / (EARTH_RADIUS * math.cos(state.initial_lat))
    
    # 3. Apply updates to the FDM structure
    fdm_data['lat_rad'] = state.initial_lat + lat_change
    fdm_data['lon_rad'] = state.initial_lon + lon_change
    
    # Set velocities for the cockpit instruments (v_body_u is forward speed)
    fdm_data['v_body_u'] = state.velocity_mps * 3.28084  # m/s to ft/s
    fdm_data['vcas'] = state.velocity_mps * 1.94384      # m/s to knots
    
    # Lock attitude for a steady runway roll
    fdm_data['phi_rad'] = 0.0    # No roll
    fdm_data['theta_rad'] = 0.0  # No pitch
    fdm_data['agl_m'] = 0.0      # Stay on ground
    
    # 4. Logging
    # Dictionary comprehension to grab all parameters for the log
    log_dict = {k: f"{v:.4f}" if isinstance(v, float) else v for k, v in fdm_data.items()}
    logger.info(f"T+{elapsed:.2f}s | Vel: {state.velocity_mps:.1f}m/s | Data: {log_dict}")

    return fdm_data

if __name__ == '__main__':
    fdm_conn = FDMConnection()
    
    # Using the standard ports for FlightGear Native FDM
    fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)

    print(f"Running smooth runway roll at {ACCEL_K} m/s^2. Check flight_log.log for data.")
    fdm_conn.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        fdm_conn.stop()
        print("\nSimulation Ended.")