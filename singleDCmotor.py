import serial
import time

PORT = "COM10"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# IMPORTANT: wait for Arduino reset
time.sleep(2)

print("Connected.")

def send_velocity(v):

    v = max(min(v, 100), -100)

    command = f"V{v}\n"
    ser.write(command.encode())

    ack = ser.readline().decode().strip()

    if ack:
        print("Arduino:", ack)

def DC_main():
    try:
        while True:
            
            velocity = int(input("Velocity (-100 to 100): "))
            send_velocity(velocity)

    except KeyboardInterrupt:

        print("Stopping motor...")
        send_velocity(0)

if __name__ == "__main__":
    DC_main()