import serial
import time
import numpy as np

PORT = "COM9" # Update as needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

time.sleep(2)  # Allow Arduino reset

# Clear buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Communication established successfully.")

# Send velocities to Arduino as a list, later would be changed to depend on Jacobian
def send_velocities(velocities):
    """
    velocities is list like [50, -30, 80]
    """
    # Create command string
    command = "D," + ",".join(map(str, velocities)) + "\n"

    # Send command over serial, encoded as bytes
    ser.write(command.encode())

    # Acknowledgement from Arduino
    if ser.in_waiting: # If there's data to read
        ack = ser.readline().decode(errors='ignore').strip() # Read line and decode
        if ack:
            print("Arduino:", ack)

def main():
    try:
        while True:
            user_input = input(
                "\nEnter velocities separated by space in m/s (example: 0.05 0 0 0)\n"
                "Press q to quit:\n> "
            )

            if user_input.lower() == 'q':
                break

            try:
                cable_velocities = np.array(list(map(float, user_input.split())))
                motor_angular_velocities = cable_velocities / (0.015)  # (rad/s)
                motor_angular_velocities_RPM = motor_angular_velocities * (60/(2*np.pi))  # Convert to RPM
                target_motor_velocities = motor_angular_velocities_RPM
                print("Target motor velocities (RPM):", target_motor_velocities)

                send_velocities(target_motor_velocities)

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