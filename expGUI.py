import tkinter as tk
import csv
from datetime import datetime
import run_trial
import read_data
# -------------------------
# Parameters
# -------------------------
RECT_W = 240
RECT_H = 440
BAR_W = 40
BAR_H = 440

CSV_POINTS_FILE = "exp0.csv" # Change to exp1.csv, exp2.csv, or exp3.csv as needed
points = read_data.load_points(CSV_POINTS_FILE)
TOTAL_TRIALS = len(points) - 1

# -------------------------
# Experiment state
# -------------------------
trial = 1
perceived_x = None
perceived_y = None
perceived_intensity = None
wait = False

# -------------------------
# CSV setup
# -------------------------
filename = f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(filename, mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["trial", "perceived_x", "perceived_y", "intensity"])

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

NUM_LEVELS = 6   # gives values 0–5

# height of each level
level_height = BAR_H / NUM_LEVELS

for i in range(1, NUM_LEVELS):
    y_line = bar_y0 + i * level_height
    canvas.create_line(
        bar_x0, 
        y_line, 
        bar_x1, 
        y_line, 
        fill="gray70", 
        width=1, 
        tags="level_lines"
    )

canvas.tag_raise(bar_fill)
canvas.tag_raise("level_lines")


for i in range(NUM_LEVELS):

    # Position text in the CENTER of each level
    y_center = bar_y1 - (i + 0.5) * level_height

    canvas.create_text(
        bar_x1 + 15,        # slightly to the right of the bar
        y_center,
        text=str(i),
        font=("Arial", 10)
    )

# Click handler
def on_click(event):
    global perceived_x, perceived_y, perceived_intensity

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
        #canvas.coords(bar_fill, bar_x0, y, bar_x1, bar_y1)
        #perceived_intensity = 1 - (y - bar_y0) / BAR_H

        # distance from TOP of bar
        relative_y = y - bar_y0

        # compute level index
        level = int(relative_y // level_height)

        # clamp (safety)
        level = max(0, min(level, NUM_LEVELS-1))

        # invert so TOP = 5, BOTTOM = 0
        perceived_intensity = (NUM_LEVELS-1) - level

        # snap fill to the level boundary
        snapped_y = bar_y0 + level * level_height

        canvas.coords(bar_fill, bar_x0, snapped_y, bar_x1, bar_y1)

# Next trial logic
def start_trial():
    """
    Starts the robot movement and stretch for the current trial.
    Called automatically at the start and after each trial.
    """
    global wait, trial

    wait = False
    next_button.config(state="disabled")  # disable until stretch finishes

    xi, yi, _ = points[trial-1]
    xf, yf, intensity = points[trial]

    def run_and_enable():
        success = run_trial.run_trial(xi, yi, xf, yf, intensity)
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
    global trial, perceived_x, perceived_y, perceived_intensity, wait

    if perceived_x is None or perceived_y is None or perceived_intensity is None:
        print("Please provide both position and intensity before continuing.")
        return

    # Log data
    csv_writer.writerow([trial, perceived_x, perceived_y, perceived_intensity])
    csv_file.flush()  # Ensure data is written immediately

    # Reset markers
    canvas.coords(rect_marker, 0, 0, 0, 0)
    canvas.coords(bar_fill, bar_x0, bar_y1, bar_x1, bar_y1)

    perceived_x = perceived_y = perceived_intensity = None
    wait = False
    trial += 1
    trial_label.config(text=f"Trial {trial}/{TOTAL_TRIALS}")

    if trial > len(points)-1:
        print("All trials done. Returning to initial position...")
        xi, yi, _ = points[-1]
        xf, yf, _ = points[0]  # Return to initial point
        run_trial.run_trial(xi, yi, xf, yf, 0, stretch=False)  # NO stretch applied
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