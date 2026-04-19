import sys
import os
import json
import time
import threading

try:
    from so101_hardware_interface.motors.feetech import FeetechMotorsBus
except ImportError:
    print("Ensure you have sourced your ROS 2 workspace (e.g. source install/setup.bash)")
    print("and run this script using: python3 src/arm_hardware_interface/arm_hardware_interface/calibrate.py")
    sys.exit(1)

def main():
    # Hardcoded IDs based on your previous config
    motor_config = {
        "shoulder_pan": {"id": 1, "drive_mode": 0},
        "shoulder_lift": {"id": 2, "drive_mode": 0},
        "elbow_flex": {"id": 3, "drive_mode": 0},
        "wrist_flex": {"id": 4, "drive_mode": 0},
        "wrist_roll": {"id": 5, "drive_mode": 0},
        "gripper": {"id": 6, "drive_mode": 0}
    }
    motor_names = list(motor_config.keys())
    motor_ids = [motor_config[m]["id"] for m in motor_names]
    motor_models = ["sts3215"] * len(motor_ids)

    print("Initializing connection to servos on /dev/ttyACM0 at 1,000,000 baud...")
    bus = FeetechMotorsBus(port="/dev/ttyACM0", motors={name: (motor_config[name]["id"], "sts3215") for name in motor_names})
    try:
        bus.connect()
        bus.set_bus_baudrate(1_000_000)
    except Exception as e:
        print(f"Failed to connect to motors: {e}")
        sys.exit(1)

    # Disable torque so joints can be moved freely
    for name in motor_names:
        bus.write("Torque_Enable", 0, name)
    
    print("\nTorque disabled. The robot should be limp.")
    input("Step 1: Move all joints to their MIDDLE / ZERO position, then press ENTER...")

    try:
        homing_positions = bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position")
    except Exception as e:
        print(f"Failed to read positions: {e}")
        bus.disconnect()
        sys.exit(1)

    for name, pos in zip(motor_names, homing_positions):
        motor_config[name]["homing_offset"] = pos
        motor_config[name]["range_min"] = pos
        motor_config[name]["range_max"] = pos

    print("\nHoming offsets saved!")
    print("Step 2: Move all joints through their FULL RANGE (minimum to maximum).")
    input("Press ENTER to start recording min/max. (Then press ENTER again when done to save) ")

    # We will run a background thread to print the table while waiting for Enter
    stop_event = threading.Event()
    
    # Track continuous position to handle encoder wrap around 4096 -> 0
    prev_positions = list(homing_positions)
    unwrapped_positions = list(homing_positions)
    
    def monitor_loop():
        while not stop_event.is_set():
            try:
                positions = bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position")
                
                # Unwrap logic for 0-4095 boundary
                for i in range(len(positions)):
                    diff = positions[i] - prev_positions[i]
                    if diff > 2048:
                        diff -= 4096
                    elif diff < -2048:
                        diff += 4096
                    unwrapped_positions[i] += diff
                    prev_positions[i] = positions[i]
                
                # Clear terminal
                os.system('cls' if os.name == 'nt' else 'clear')
                print("--- Calibration: Min/Max Recording ---")
                print("Move the robot joints around. Press ENTER to stop and save.\n")
                print(f"{'Joint Name':<15} | {'Current':<8} | {'Min':<8} | {'Max':<8}")
                print("-" * 50)
                
                for i, name in enumerate(motor_names):
                    pos = unwrapped_positions[i]
                    config = motor_config[name]
                    if pos < config["range_min"]:
                        config["range_min"] = pos
                    if pos > config["range_max"]:
                        config["range_max"] = pos
                        
                    print(f"{name:<15} | {pos:<8} | {config['range_min']:<8} | {config['range_max']:<8}")
                
                time.sleep(0.1) # 10Hz UI refresh
            except Exception:
                pass

    monitor_thread = threading.Thread(target=monitor_loop)
    monitor_thread.daemon = True
    monitor_thread.start()

    # Wait for the user to press Enter to stop
    input()
    stop_event.set()
    monitor_thread.join()

    print("\nSaving calibration data...")
    bus.disconnect()

    # Determine file path
    package_src_dir = os.path.join(os.getcwd(), "src", "arm_hardware_interface")
    if not os.path.exists(package_src_dir):
        # Fallback if run from the package dir itself
        package_src_dir = os.getcwd()
        
    resource_dir = os.path.join(package_src_dir, "resource")
    os.makedirs(resource_dir, exist_ok=True)
    
    json_path = os.path.join(resource_dir, "so101_calibration.json")
    
    with open(json_path, 'w') as f:
        json.dump(motor_config, f, indent=4)
        
    print(f"\nCalibration saved successfully to: {json_path}")
    print("You can verify the values in the JSON file.")

if __name__ == '__main__':
    main()
