import pypot.dynamixel as pd
import time

# get ports and start connection to motors
ports = pd.get_available_ports()
if not ports:
    print("No serial ports found.")
    quit()

motors = pd.Dxl320IO(ports[0], 1000000)

motor_ids = motors.scan(range(20))

# only work with one motor at a time
if len(motor_ids) > 1:
    print("Only connect one motor at a time!")
    quit()

if len(motor_ids) < 1:
    print("No connected motors found! Did you remember to connect external power?")
    quit()

motor_id = motor_ids[0]
print("Motor " + str(motor_id) + " found!")

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
            else:
                print("Sorry, ID change failed. Try unplugging and reconnecting the motor.")
                quit()
        except ValueError as e:
            print("Error:", e)
            quit()
    else:
        print("New ID is the same as current. Skipping change.")

# Calibration, step by step
def set_position_and_wait(position, message):
    print(f"Setting position to {position}...")
    motors.set_goal_position({motor_id: position})
    input(message)

set_position_and_wait(100, "Motor position: 100. Attach horn then press 'Enter'. ")
set_position_and_wait(0, "Motor position: 0. Calibrate string length then press 'Enter'. ")
set_position_and_wait(100, "Motor position: 100. Calibration complete!")

motors.close()
