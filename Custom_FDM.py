import time
import math
import logging
from logging.handlers import RotatingFileHandler
from flightgear_python.fg_if import FDMConnection

# --- LOGGER SETUP ---
logger = logging.getLogger("FlightDynamicsLogger")
logger.setLevel(logging.INFO)
# Rotating log file: max 5MB per file, keeps last 5 backups
handler = RotatingFileHandler("flight_log.log", maxBytes=5*1024*1024, backupCount=5)
formatter = logging.Formatter('%(asctime)s | %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# --- SIMULATION CONSTANTS ---
ACCEL_K = 3.0  # m/s^2 (The uniform acceleration)
EARTH_RADIUS = 6378137.0  # meters
FPS = 30
DT = 1.0 / FPS

class AircraftState:
    def __init__(self):
        self.velocity_mps = 0.0
        self.distance_m = 0.0
        self.start_time = None
        self.initial_lat = None
        self.initial_lon = None

state = AircraftState()

def fdm_callback(fdm_data, event_pipe):
    curr_time = time.time()
    
    # Initialize start position and time on first packet
    if state.start_time is None:
        state.start_time = curr_time 
        state.initial_lat = fdm_data['lat_rad']
        state.initial_lon = fdm_data['lon_rad']
        logger.info("Simulation Started - Initial Position Logged")

    elapsed = curr_time - state.start_time

    # 1. Uniform Acceleration Logic (for 10 seconds)
    if elapsed <= 10.0:
        # v = u + at
        state.velocity_mps += ACCEL_K * DT
        # s = ut + 0.5at^2 (calculated incrementally here)
        state.distance_m += state.velocity_mps * DT
    else:
        # After 10s, maintain constant velocity for stability
        pass

    # 2. Update Position (Moving Straight - Northward for simplicity)
    # To move straight, we update Latitude based on distance traveled
    new_lat = state.initial_lat + (state.distance_m / EARTH_RADIUS)
    
    # 3. Update FDM Data Structure
    fdm_data['lat_rad'] = new_lat
    fdm_data['v_body_u'] = state.velocity_mps * 3.28084  # Convert m/s to ft/s
    fdm_data['vcas'] = state.velocity_mps * 1.94384      # Convert m/s to Knots
    
    # Ensure aircraft stays on ground and level
    fdm_data['phi_rad'] = 0.0    # No Roll
    fdm_data['theta_rad'] = 0.0  # No Pitch (Keep nose down on runway)
    fdm_data['agl_m'] = 0.0      # At Ground Level

    # 4. Logging All Dynamics
    # We convert the fdm_data object to a dict to log all parameters
    log_msg = ", ".join([f"{k}: {v}" for k, v in fdm_data.items()])
    logger.info(f"Time: {elapsed:.2f}s | Dist: {state.distance_m:.1f}m | {log_msg}")

    return fdm_data

if __name__ == '__main__':
    # Using the ports provided in your example
    fdm_conn = FDMConnection()
    
    # RX: Receiving/Setting data to FlightGear
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    
    # TX: Listening to FlightGear (if needed)
    fdm_conn.connect_tx('localhost', 5502)

    print(f"Starting runway roll with acceleration {ACCEL_K} m/s^2...")
    fdm_conn.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping Simulation and closing logs.")
        fdm_conn.stop()