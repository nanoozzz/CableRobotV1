import serial
import time

PORT = "COM10"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.2)

time.sleep(2)  # allow Arduino reset

ser.reset_input_buffer()
ser.reset_output_buffer()

print("Connected.")


def clamp(v):
    return max(min(int(v), 100), -100)


def send_velocities(velocities):
    """
    velocities → list like [50, -30, 80]
    """

    velocities = [clamp(v) for v in velocities]

    command = "D," + ",".join(map(str, velocities)) + "\n"

    ser.write(command.encode())

    # Optional acknowledgement
    if ser.in_waiting:
        ack = ser.readline().decode(errors='ignore').strip()
        if ack:
            print("Arduino:", ack)


def DC_main():

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

                send_velocities(velocities)

            except Exception as e:
                print("Invalid input:", e)


    except KeyboardInterrupt:
        pass

    finally:
        print("Stopping ALL motors...")
        send_velocities([0])   # Arduino can interpret missing motors as 0
        time.sleep(0.2)
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    DC_main()