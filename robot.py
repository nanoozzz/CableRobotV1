import serial
import time
import numpy as np
import read_data

# Robot constants 
b = 66.464e-3
L = 300.0e-3
W = 200.0e-3

ACCELERATION = 0.8  # m/s^2

CONTROL_RATE = 100  # Hz
dt = 1.0 / CONTROL_RATE  # Time step
RPM_TO_TICKS_PER_REV = 48*74.83/60
BREAKAWAY_CMD = np.array([50, 50, 50, 50])  # Minimum command to overcome motor deadzone

ser = None # Global serial object, initialized in init_serial() and closed in close_serial()

# Starting position
x = 100.0e-3
y = 150.0e-3

PORT = "COM9" # Update as needed
BAUD = 115200

time.sleep(2)  # Allow Arduino reset

# Initialize serial communication
def init_serial():
    global ser

    if ser is not None:
        return ser
    
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print("Communication established successfully.\n")
    return ser

# Close serial communication safely
def close_serial():
    """
    Safely stop all motors and close the serial connection.
    """
    global ser

    if ser is not None:
        print("Stopping all motors and closing serial...")

        try:
            # Stop motors
            send_velocities([0, 0, 0, 0])
            time.sleep(0.2)  # Ensure command is sent

            # Close serial
            ser.close()
            print("Serial port closed safely.")

        except Exception as e:
            print("Error closing serial:", e)

        finally:
            ser = None  # Reset so it can be reinitialized later

# Send velocities to Arduino as a list, later would be changed to depend on Jacobian
def send_velocities(velocities):
    """
    velocities is list like [v1, v2, v3, v4]
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

# Compute Jacobian matrix
def jacobian(x,y,b,W,L):
    eps = 1e-6  # Small value to prevent division by zero

    def safe_norm(dx,dy): # Safe norm to avoid division by zero
        return max(np.sqrt(dx**2 + dy**2), eps)
    
    J = np.array([
        # Cable 1
        [
            (x - b/2) / safe_norm(x - b/2, y - b/2),
            (y - b/2) / safe_norm(x - b/2, y - b/2)
        ],

        # Cable 2
        [
            (x + b/2 - W) / safe_norm(x + b/2 - W, y - b/2),
            (y - b/2) / safe_norm(x + b/2 - W, y - b/2)
        ],

        # Cable 3
        [
            (x + b/2 - W) / safe_norm(x + b/2 - W, y + b/2 - L),
            (y + b/2 - L) / safe_norm(x + b/2 - W, y + b/2 - L)
        ],

        # Cable 4
        [
            (x - b/2) / safe_norm(x - b/2, y + b/2 - L),
            (y + b/2 - L) / safe_norm(x - b/2, y + b/2 - L)
        ]
    ])

    return J

# Quintic polynomial coefficients for smooth trajectory, initial and final velocities and accelerations are zero
def quintic_coefficients(xi, xf, tf):
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0 ,0 ,0],
        [0, 0, 1, 0, 0, 0],
        [1, tf, tf**2, tf**3,tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
    ])
    b_vec = np.array([xi, 0, 0, xf, 0, 0])
    return np.linalg.solve(A, b_vec)

# Evaluate quintic polynomial at time t
def evaluate_position(c, t):
    return (c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5)

# Evaluate velocity from quintic polynomial at time t
def evaluate_velocity(c, t):
    return (c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4)

# Execute trajectory from (xi, yi) to (xf, yf) with max acceleration a
def execute_trajectory(xi, yi, xf, yf, a = ACCELERATION):
    dx = xf - xi
    dy = yf - yi
    tf = max(np.sqrt(2*abs(dx)/a),
             np.sqrt(2*abs(dy)/a),
             dt)  # Ensure tf is at least dt to avoid zero duration
    print(f"Moving ({xi:.2f},{yi:.2f}) → ({xf:.2f},{yf:.2f}) | tf={tf:.2f}s") # Debug only

    # Solve the quintic coefficients for x and y
    cx = quintic_coefficients(xi, xf, tf)
    cy = quintic_coefficients(yi, yf, tf)

    t = 0.0
    next_time = time.time() 

    try:
        while t <= tf:
            next_time += dt

            # Evaluate trajectory
            x = evaluate_position(cx, t)
            y = evaluate_position(cy, t)

            vx = evaluate_velocity(cx, t)
            vy = evaluate_velocity(cy, t)

            ee_vel = np.array([vx, vy])
            J = jacobian(x, y, b, W, L)

            cable_velocities = J @ ee_vel  # Matrix multiplication

            # Testing only, overwrite cable_velocities
            #cable_velocities = np.array([0.1, 0, 0, 0])
            #motor_angular_velocities = cable_velocities / (0.015)  # (rad/s)
            #motor_angular_velocities_RPM = motor_angular_velocities * (60/(2*np.pi))  # Convert to RPM
            #target_motor_velocities = motor_angular_velocities_RPM *K  # Convert to ticks/ms
            ###

            motor_angular_velocities = cable_velocities / (0.015)  # (rad/s)
            motor_angular_velocities_RPM = motor_angular_velocities * (60/(2*np.pi))  # Convert to RPM
            target_motor_velocities = motor_angular_velocities_RPM   
            
            #PRINT_EVERY = 2  # Print every N iterations, debug only
            #if int(t / dt) % PRINT_EVERY == 0:
            #    print(f"t={t:.2f}s | Cable Velocities={cable_velocities} | Motor Commands={target_motor_velocities}")  # Debug only
            
            send_velocities(target_motor_velocities)

            # Maintain control rate
            sleep_duration = next_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            
            t += dt
        print("Trajectory execution completed.")

    except KeyboardInterrupt:
        pass  # Allow graceful exit on Ctrl+C

    finally:
        print("Stopping ALL motors...")
        send_velocities([0, 0, 0, 0])   # Arduino can interpret missing motors as 0
        time.sleep(0.2)
    
# Run trajectory through points loaded from CSV file
def run_trajectory(csv_file):
    points = read_data(csv_file)

    print("Loaded points:") # Debug only
    for p in points:
        print(f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})") # Debug only
    
    initial_point = points[0]

    try:
        for i in range(len(points)-1):
            xi, yi, intensity = points[i]
            xf, yf, _ = points[i+1]

            execute_trajectory(xi, yi, xf, yf, a = ACCELERATION)

            print("Waiting 10 seconds on user's response...\n")
            time.sleep(10)
        
        last_point = points[-1]
        print("Returning to initial point...")
        execute_trajectory(last_point[0], last_point[1], initial_point[0], initial_point[1], a = ACCELERATION)
        print("All trajectories completed.")

    except Exception as e:
        print("Error in trajectory execution:", e)

    finally:
        send_velocities([0, 0, 0, 0])   # Stop all motors

# Run robot with given CSV file
def run_robot(csv_file):
    global ser

    ser = init_serial()
    try:
        run_trajectory(csv_file)

    finally:
        close_serial()

# Main function
def main():
    csv_file = input("Enter CSV filename:\n> ")
    run_robot(csv_file)

if __name__ == "__main__":
    main()