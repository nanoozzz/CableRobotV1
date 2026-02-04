import robot
import Stretch
import time

CSV_FILE = "points.csv"
STRETCH_OFFSET = 342
PAUSE_AFTER_MOVE = 1.5   # seconds (let robot settle)

def run_trial(xi, yi, xf, yf,stretch = True):
    print("Trial starts\n")

    try:
        robot.init_serial()
        robot.execute_trajectory(xi, yi, xf, yf)
        time.sleep(PAUSE_AFTER_MOVE)

        if stretch:
            print("\nApplying stretch...")
            Stretch.run_stretch(STRETCH_OFFSET)
            print("Stretch applied.")
        return True

    except KeyboardInterrupt:
        print("\nTrial interrupted by user.")

    except Exception as e:
        print("\nTRIAL ERROR:", e)

    finally:
        robot.close_serial()

if __name__ == "__main__":
    run_trial(0, 0, 0, 0)
