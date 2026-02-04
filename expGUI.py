import tkinter as tk
import csv
from datetime import datetime
import runTrial
import robot
# -------------------------
# Parameters
# -------------------------
RECT_W = 240
RECT_H = 440
BAR_W = 40
BAR_H = 440
TOTAL_TRIALS = 3
CSV_POINTS_FILE = "points.csv"

# -------------------------
# Experiment state
# -------------------------
trial = 1
perceived_x = None
perceived_y = None
intensity = None
wait = False
points = robot.load_points(CSV_POINTS_FILE)

# -------------------------
# CSV setup
# -------------------------
filename = f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(filename, mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "trial",
    "perceived_x", "perceived_y",
    "intensity"
])

# GUI setup
root = tk.Tk()
root.title("Cable Robot Perception Study")

canvas = tk.Canvas(root, width=400, height=600)
canvas.pack(padx=10, pady=10)

# Trial label
trial_label = tk.Label(root, text=f"Trial {trial}/{TOTAL_TRIALS}", font=("Arial", 12))
trial_label.pack()

# Rectangle workspace
rect_x0, rect_y0 = 20, 40
rect_x1, rect_y1 = rect_x0 + RECT_W, rect_y0 + RECT_H
canvas.create_rectangle(rect_x0, rect_y0, rect_x1, rect_y1, outline="black", width=2)
canvas.create_text((rect_x0 + rect_x1) / 2, rect_y0 - 12, text="Position")

rect_marker = canvas.create_oval(0, 0, 0, 0, fill="red")

# Intensity bar
bar_x0 = rect_x1 + 40
bar_y0 = rect_y0
bar_x1 = bar_x0 + BAR_W
bar_y1 = bar_y0 + BAR_H
canvas.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, outline="black", width=2)
canvas.create_text(bar_x0 + BAR_W / 2, bar_y0 - 12, text="Intensity")

bar_fill = canvas.create_rectangle(bar_x0, bar_y1, bar_x1, bar_y1, fill="blue")

# Click handler
def on_click(event):
    global perceived_x, perceived_y, intensity

    x, y = event.x, event.y

    if wait is False:
        print("Please wait for stretch to complete.")
        return
    
    # Workspace click
    if rect_x0 <= x <= rect_x1 and rect_y0 <= y <= rect_y1:
        r = 5
        canvas.coords(rect_marker, x-r, y-r, x+r, y+r)

        perceived_x = (x - rect_x0) / RECT_W
        perceived_y = 1 - (y - rect_y0) / RECT_H

    # Intensity bar click
    elif bar_x0 <= x <= bar_x1 and bar_y0 <= y <= bar_y1:
        canvas.coords(bar_fill, bar_x0, y, bar_x1, bar_y1)
        intensity = 1 - (y - bar_y0) / BAR_H

# Next trial logic
def start_trial():
    """
    Starts the robot movement and stretch for the current trial.
    Called automatically at the start and after each trial.
    """
    global wait, trial
    """
    if trial > len(points)-1:
        print("All trials completed.")
        csv_file.close()
        root.destroy()
        return"""

    wait = False
    next_button.config(state="disabled")  # disable until stretch finishes

    xi, yi = points[trial-1]
    xf, yf = points[trial]

    def run_and_enable():
        success = runTrial.run_trial(xi, yi, xf, yf)
        if success:
            print("Stretch complete. You may now click the workspace.")
        else:
            print("Trial failed.")
        global wait
        wait = True
        next_button.config(state="normal")

    # Use Tkinter after() to avoid freezing GUI
    root.after(100, run_and_enable)

# Next trial logic
def next_trial():
    global trial, perceived_x, perceived_y, intensity, wait

    if perceived_x is None or perceived_y is None or intensity is None:
        print("Please provide both position and intensity before continuing.")
        return

    # Log data
    csv_writer.writerow([
        trial,
        perceived_x, perceived_y,
        intensity
    ])

    # Reset markers
    canvas.coords(rect_marker, 0, 0, 0, 0)
    canvas.coords(bar_fill, bar_x0, bar_y1, bar_x1, bar_y1)

    perceived_x = perceived_y = intensity = None
    wait = False
    trial += 1
    trial_label.config(text=f"Trial {trial}/{TOTAL_TRIALS}")

    """
    if trial > TOTAL_TRIALS:
        csv_file.close()
        root.destroy()
        print("Experiment finished.")
        return """

    if trial > len(points)-1:
        print("All trials done. Returning to initial position...")
        xi, yi = points[-1]
        xf, yf = points[0]  # Return to initial point
        runTrial.run_trial(xi, yi, xf, yf, stretch=False)  # NO stretch applied
        print("Robot returned home. Experiment finished.")
        csv_file.close()
        root.destroy()
    else:
        # Start next robot trial
        start_trial()

# Buttons and bindings
canvas.bind("<Button-1>", on_click)

next_button = tk.Button(root, text="Next Trial", command=next_trial, state="disabled")
next_button.pack(pady=10)

def main():
    start_trial()  # Start the first trial automatically
    root.mainloop()

if __name__ == "__main__":
    main()