import pypot.dynamixel as pd
import time

# Get ports and connect
ports = pd.get_available_ports()
if not ports:
    print("No serial ports found.")
    quit()

motors = pd.Dxl320IO(ports[0], 1_000_000)
motor_ids = motors.scan(range(1, 20))

if len(motor_ids) < 1:
    print("No connected motors found! Did you remember to connect external power?")
    quit()

EXPECTED_IDS = [1, 2, 3, 4, 5]
only_one_motor = len(motor_ids) == 1

if only_one_motor:
    motor_id = motor_ids[0]
    print(f"Motor {motor_id} found!")

    new_id_str = input("Enter new motor ID (1-3 for towers, 4 for base, 5+ for etc). Press Enter to keep current ID: ")

    if new_id_str != '':
        new_id = int(new_id_str)
        if new_id != motor_id:
            motors.disable_torque([motor_id])
            print(f"Changing motor ID from {motor_id} to {new_id}")
            try:
                motors.change_id({motor_id: new_id})
                time.sleep(0.2)
                if motors.ping(new_id):
                    motor_id = new_id
                    print(f"ID successfully changed to {new_id}")
                else:
                    print("ID change failed. Try unplugging and reconnecting the motor.")
                    quit()
            except ValueError as e:
                print("Error:", e)
                quit()
        else:
            print("New ID is the same as current. Skipping change.")
else:
    print("Detected motors:", motor_ids)
    if sorted(motor_ids) != EXPECTED_IDS:
        print("Expected motors with IDs 1 to 5. Please connect all 5 motors.")
        quit()
    else:
        print("All 5 motors detected. Starting calibration...")

# --- Calibration functions ---

def set_position_and_wait(mid, position, message):
    print(f"[ID {mid}] → Setting position to {position}...")
    motors.set_goal_position({mid: position}) # type: ignore
    input(message)

def calibrate(mid):
    print(f"\n==> Calibrating motor {mid}")
    set_position_and_wait(mid, 100, "Attach horn at 100. Then press Enter.")
    set_position_and_wait(mid, 0, "Motor moved to 0. Adjust string tension, then press Enter.")
    set_position_and_wait(mid, 100, "Motor returned to 100. Calibration complete.")

def calibrate_ear(mid):
    print(f"\n==> Calibrating ear motor {mid}")
    motors.set_goal_position({mid: 100}) # type: ignore
    input("Position 100. Attach horn, then press Enter.")
    motors.set_goal_position({mid: 150})# type: ignore
    input("Position 150. Tighten string, then press Enter to finish.")

# --- Run calibration ---
if only_one_motor:
    calibrate(motor_ids[0])
else:
    for motor_id in [1, 2, 3]: # tower motors
        calibrate(motor_id)
    calibrate(4)      # base
    calibrate_ear(5)  # ears

motors.close()

