from dynamixel_sdk import *
import time
import sys

############################################################
# Control Table (XL-330)
############################################################

ADDR_OPERATING_MODE      = 11
ADDR_TORQUE_ENABLE       = 64
ADDR_PROFILE_ACCEL       = 108
ADDR_PROFILE_VELOCITY    = 112
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_POSITION    = 132

PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM14' # Update as needed

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
POSITION_MODE  = 3

# Utility Functions
# Check communication results
def check_comm(packetHandler, result, error):
    if result != COMM_SUCCESS: # Communication error
        raise Exception(packetHandler.getTxRxResult(result))
    elif error != 0: # Dynamixel error
        raise Exception(packetHandler.getRxPacketError(error))

# Wait until motor reaches goal position
def wait_until_arrived(portHandler, packetHandler, goal, timeout=10):
    """
    Wait until the motor reaches the goal position.
    Includes timeout to prevent infinite loop.
    """

    start = time.time() 
    threshold = 5   # 1 tick = 0.088 deg 

    while True:
        present, result, error = packetHandler.read4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRESENT_POSITION) # Current position

        check_comm(packetHandler, result, error)

        print(f"Present: {present} | Goal: {goal}")

        if abs(goal - present) < threshold: # Prevent infinite loop
            break

        if time.time() - start > timeout: # Timeout
            raise Exception("Motor motion timeout")

        time.sleep(0.05)

# Setup
def setup_motor():

    portHandler = PortHandler(DEVICENAME) # Open port
    packetHandler = PacketHandler(PROTOCOL_VERSION) # Create packet handler

    if not portHandler.openPort():
        raise Exception("Failed to open port")

    print("Port opened.")

    if not portHandler.setBaudRate(BAUDRATE):
        raise Exception("Failed to set baudrate")

    print("Baudrate set.")

    # Disable torque BEFORE mode change
    result, error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    check_comm(packetHandler, result, error)

    # Set Position Mode
    result, error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_OPERATING_MODE, POSITION_MODE)

    check_comm(packetHandler, result, error)

    # Motion Profile
    # Lower = smoother + safer

    packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_PROFILE_ACCEL, 50)

    packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_PROFILE_VELOCITY, 50)

    # Prevent startup jump
    current_pos, result, error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION)

    check_comm(packetHandler, result, error)

    packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_GOAL_POSITION, current_pos)

    # Enable torque
    result, error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    check_comm(packetHandler, result, error)

    print("Motor initialized safely.")
    print("Zero position:", current_pos)

    return portHandler, packetHandler, current_pos

# Safe Shutdown 
def shutdown(portHandler, packetHandler):

    print("\nShutting down safely...")

    packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()

    print("Torque disabled. Port closed.")

def run_stretch(offset):
    portHandler = None
    try:
        
        portHandler, packetHandler, zero = setup_motor()
        # Choose target offset
        #offset = 342   # 0-4095 is 1 rev, 1 ~ 0.229 rpm
        goal = max(0, min(4095, zero + offset)) # ~30 degrees

        print("\nMoving to target...")
        packetHandler.write4ByteTxRx(
            portHandler, DXL_ID, ADDR_GOAL_POSITION, goal)

        wait_until_arrived(portHandler, packetHandler, goal)

        time.sleep(1)

        print("\nReturning to zero...")
        packetHandler.write4ByteTxRx(
            portHandler, DXL_ID, ADDR_GOAL_POSITION, zero)

        wait_until_arrived(portHandler, packetHandler, zero)

        print("\nStretch completed.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        pass

    except Exception as e:
        print("\nERROR:", e)

    finally:
        if portHandler:
            shutdown(portHandler, packetHandler)

def main():
    run_stretch()

if __name__ == "__main__":
    main()
