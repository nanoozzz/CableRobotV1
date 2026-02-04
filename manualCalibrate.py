import serial
import time
import numpy as np

PORT = "COM10" # Update as needed
BAUD = 115200

BREAKAWAY_CMD = np.array([50, 50, 50, 50])

ser = serial.Serial(PORT, BAUD, timeout=1)

time.sleep(2)  # Allow Arduino reset

# Clear buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Communication established successfully.")

# Clamp function to limit velocities between -100 and 100
def clamp(v):
    return max(min(int(v), 100), -100)

# Send velocities to Arduino as a list, later would be changed to depend on Jacobian
def send_velocities(velocities):
    """
    velocities is list like [50, -30, 80]
    """

    velocities = [clamp(v) for v in velocities]


    # Create command string
    command = "D," + ",".join(map(str, velocities)) + "\n"

    # Send command over serial, encoded as bytes
    ser.write(command.encode())

    # Acknowledgement from Arduino
    if ser.in_waiting: # If there's data to read
        ack = ser.readline().decode(errors='ignore').strip() # Read line and decode
        if ack:
            print("Arduino:", ack)

def compensate_deadband(cmd, breakaway=BREAKAWAY_CMD):
    """
    Smoothly remap motor command so anything nonzero
    exceeds the breakaway threshold.

    Keeps control continuous (no jumps).
    """

    cmd = np.array(cmd, dtype=float)

    for i in range(len(cmd)):

        if cmd[i] == 0:
            continue

        sign = np.sign(cmd[i])
        mag = abs(cmd[i])

        # Remap 0–100 → breakaway–100
        mag = breakaway[i] + (100 - breakaway[i]) * (mag / 100)

        cmd[i] = sign * mag

    return cmd

def main():
    try:
        while True:
            user_input = input(
                "\nEnter velocities separated by space (example: 50 -30 80)\n"
                "Press q to quit:\n> "
            )

            if user_input.lower() == 'q':
                break

            try:
                velocities = user_input.split()
                print("Raw velocities:", velocities)
                motor_cmd = compensate_deadband(velocities, breakaway=BREAKAWAY_CMD)
                print("Compensated motor commands:", motor_cmd)

                send_velocities(motor_cmd)
            except Exception as e:
                print("Invalid input:", e)

    except KeyboardInterrupt:
        pass # Graceful exit on Ctrl+C

    finally:
        print("Stopping ALL motors...")
        send_velocities([0,0,0,0])   # Arduino can interpret missing motors as 0
        time.sleep(0.2)
        ser.close()
        print("Serial closed.")

if __name__ == "__main__":
    main()