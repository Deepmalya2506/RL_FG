import time
import math
import logging
from logging.handlers import RotatingFileHandler
from flightgear_python.fg_if import FDMConnection

# --- LOGGER SETUP ---
logger = logging.getLogger("FlightDynamicsLogger")
logger.setLevel(logging.INFO)
handler = RotatingFileHandler("flight_log_full.log", maxBytes=5*1024*1024, backupCount=5)
formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# --- SIMULATION CONSTANTS ---
ACCEL_K = 2.5            # m/s^2
TARGET_ALT_M = 300.0     # ~1000 feet
CLIMB_PITCH_DEG = 10.0   # Pitch during climb
PHASE_1_DURATION = 10.0  # Ground roll time
ROTATION_DURATION = 4.0  # Time to lift nose
LEVEL_OFF_DURATION = 6.0 # Time to transition to level flight

EARTH_RADIUS = 6378137.0 
DT = 1.0 / 30.0          # Fixed Delta Time for physics stability

class AircraftState:
    def __init__(self):
        self.velocity_mps = 0.0
        self.alt_m = 0.0
        self.pitch_rad = 0.0
        self.lat_rad = None
        self.lon_rad = None
        self.heading_rad = None
        self.start_time = None
        self.current_phase = "GROUND_ROLL"

state = AircraftState()

def fdm_callback(fdm_data, event_pipe):
    curr_time = time.time()
    
    # 1. INITIALIZATION (Frame 0)
    if state.start_time is None:
        state.start_time = curr_time
        state.lat_rad = fdm_data['lat_rad']
        state.lon_rad = fdm_data['lon_rad']
        state.heading_rad = fdm_data['psi_rad'] # LOCK HEADING
        state.alt_m = fdm_data['alt_m']
        logger.info(f"STARTING: Phase 1 (Ground Roll) at Heading {math.degrees(state.heading_rad):.1f}°")

    elapsed = curr_time - state.start_time

    # 2. STATE MACHINE & PHASE LOGIC
    # PHASE 1: Ground Roll
    if elapsed < PHASE_1_DURATION:
        state.current_phase = "GROUND_ROLL"
        state.velocity_mps += ACCEL_K * DT
        state.pitch_rad = 0.0

    # PHASE 2: Rotation & Climb
    elif state.alt_m < TARGET_ALT_M:
        if state.current_phase == "GROUND_ROLL":
            logger.info("TRANSITION: Phase 2 (Rotation/Climb)")
        state.current_phase = "CLIMB"
        
        state.velocity_mps += (ACCEL_K * 0.5) * DT # Reduced accel during climb
        
        # Smoothly increase pitch to CLIMB_PITCH_DEG
        rotation_elapsed = elapsed - PHASE_1_DURATION
        target_pitch = math.radians(CLIMB_PITCH_DEG)
        if rotation_elapsed < ROTATION_DURATION:
            # Linear interpolation for smooth nose lift
            state.pitch_rad = (rotation_elapsed / ROTATION_DURATION) * target_pitch
        else:
            state.pitch_rad = target_pitch

    # PHASE 3: Level Off
    else:
        if state.current_phase == "CLIMB":
            logger.info("TRANSITION: Phase 3 (Leveling Off)")
            state.level_start_time = elapsed
        state.current_phase = "LEVEL_FLIGHT"
        
        # Maintain velocity, smoothly return pitch to 0.0 (Cruise)
        level_elapsed = elapsed - state.level_start_time
        start_pitch = math.radians(CLIMB_PITCH_DEG)
        if level_elapsed < LEVEL_OFF_DURATION:
            state.pitch_rad = start_pitch * (1 - (level_elapsed / LEVEL_OFF_DURATION))
        else:
            state.pitch_rad = 0.0
        
        # Lock altitude strictly once leveled
        state.alt_m = TARGET_ALT_M

    # 3. INCREMENTAL PHYSICS INTEGRATION (The "Jitter Fix")
    v_vertical = state.velocity_mps * math.sin(state.pitch_rad)
    v_horizontal = state.velocity_mps * math.cos(state.pitch_rad)

    # Calculate distance moved in this specific 1/30th of a second
    dist_this_frame = v_horizontal * DT
    
    # Update Latitude/Longitude incrementally
    state.lat_rad += (dist_this_frame * math.cos(state.heading_rad)) / EARTH_RADIUS
    state.lon_rad += (dist_this_frame * math.sin(state.heading_rad)) / (EARTH_RADIUS * math.cos(state.lat_rad))
    
    # Update Altitude (only if not finished leveling off)
    if state.current_phase != "LEVEL_FLIGHT" or state.pitch_rad > 0:
        state.alt_m += v_vertical * DT

    # 4. UPDATE FDM OBJECT
    fdm_data['lat_rad'] = state.lat_rad
    fdm_data['lon_rad'] = state.lon_rad
    fdm_data['alt_m'] = state.alt_m
    fdm_data['theta_rad'] = state.pitch_rad
    fdm_data['psi_rad'] = state.heading_rad  # FORCE HEADING LOCK
    fdm_data['phi_rad'] = 0.0               # FORCE WINGS LEVEL
    
    # Airspeed and Climb Rate for cockpit
    fdm_data['v_body_u'] = state.velocity_mps * 3.28084
    fdm_data['vcas'] = state.velocity_mps * 1.94384
    fdm_data['climb_rate_ft_per_s'] = v_vertical * 3.28084

    # 5. DATA LOGGING
    logger.info(f"[{state.current_phase}] Time:{elapsed:.2f}s | Alt:{state.alt_m:.1f}m | Pitch:{math.degrees(state.pitch_rad):.1f}° | Spd:{fdm_data['vcas']:.1f}kt")

    return fdm_data

if __name__ == '__main__':
    fdm_conn = FDMConnection()
    fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)

    print("System Online. Executing Flight Plan: Roll -> Rotate -> Level.")
    fdm_conn.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fdm_conn.stop()
        print("\nFlight Logged and Connection Closed.")