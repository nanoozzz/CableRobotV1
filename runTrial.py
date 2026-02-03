import robot
import Stretch
import time

CSV_FILE = "points.csv"
STRETCH_OFFSET = 342
PAUSE_AFTER_MOVE = 1.5   # seconds (let robot settle)

def main():
    print("Trial starts\n")

    robot.init_serial()

    try:

        # Load trajectory points
        points = robot.load_points(CSV_FILE)

        initial_point = points[0]

        print("\nLoaded points:")
        for p in points:
            print(p)

        print("\nStarting trials...\n")

        # Move through each segment
        for i in range(len(points) - 1):

            xi, yi = points[i]
            xf, yf = points[i + 1]

            print(f"\n--- Move {i+1}: ({xi:.3f},{yi:.3f}) to ({xf:.3f},{yf:.3f}) ---")

            robot.execute_trajectory(xi, yi, xf, yf)

            time.sleep(PAUSE_AFTER_MOVE)

            print("\nApplying stretch...")
            Stretch.run_stretch(STRETCH_OFFSET)

            print("\nStretch complete.")

        # Return home WITHOUT stretch
        print("\nReturning to initial position (no stretch)...")

        last_point = points[-1]

        robot.execute_trajectory(
            last_point[0],
            last_point[1],
            initial_point[0],
            initial_point[1]
        )

        print("\n========== TRIAL COMPLETE ==========\n")


    except KeyboardInterrupt:
        print("\nTrial interrupted by user.")

    except Exception as e:
        print("\nTRIAL ERROR:", e)

    finally:
        robot.close_serial()

if __name__ == "__main__":
    main()