import tkinter as tk
import threading
from datetime import datetime
import read_data
import run_trial

# -------------------------
# Parameters
# -------------------------
RECT_W = 200
RECT_H = 300
BAR_W = 40
BAR_H = 300
WORKSPACE_WIDTH = 0.2
WORKSPACE_HEIGHT = 0.3
OFFSET_X = 10.351/1000
OFFSET_Y = 0.0
EE_SIZE = 66
TOTAL_TRIALS = 10
CSV_POINTS_FILE = "demo.csv"

EE_CENTER_SIZE = 3  # half length of each arm of the X

points = read_data.load_points(CSV_POINTS_FILE)
TOTAL_TRIALS = len(points) - 1

trial = 1

# GUI setup
root = tk.Tk()
root.title("Training Demo")

canvas = tk.Canvas(root, width=400, height=400)
canvas.pack(padx=10, pady=10)

trial_label = tk.Label(root, font=("Arial", 12))
trial_label.pack()

# Rectangle workspace
rect_x0, rect_y0 = 20, 40
rect_x1, rect_y1 = rect_x0 + RECT_W, rect_y0 + RECT_H
canvas.create_rectangle(rect_x0, rect_y0, rect_x1, rect_y1, outline="black", width=2)
canvas.create_text((rect_x0 + rect_x1) / 2, rect_y0 - 12, text="Position")

ee_square = canvas.create_rectangle(0, 0, 0, 0, outline="blue", width=2)
robot_marker = canvas.create_oval(0, 0, 0, 0, fill="red")
canvas.tag_raise(robot_marker)

ee_center_line1 = canvas.create_line(0, 0, 0, 0, width=2, fill="black")
ee_center_line2 = canvas.create_line(0, 0, 0, 0, width=2, fill="black")

rect_x2, rect_y2 = 70, 90
rect_x3, rect_y3 = rect_x2 + 100, rect_y2 + 200
canvas.create_rectangle(rect_x2, rect_y2, rect_x3, rect_y3, outline="green", width=2)

# Intensity bar
bar_x0 = rect_x1 + 40
bar_y0 = rect_y0
bar_x1 = bar_x0 + BAR_W
bar_y1 = bar_y0 + BAR_H

canvas.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, outline="black", width=2)
canvas.create_text(bar_x0 + BAR_W / 2, bar_y0 - 12, text="Intensity")

bar_fill = canvas.create_rectangle(bar_x0, bar_y1, bar_x1, bar_y1, fill="blue")

NUM_LEVELS = 6   # gives values 0–5
level_height = BAR_H / NUM_LEVELS

for i in range(1, NUM_LEVELS):
    y = bar_y0 + i * level_height
    canvas.create_line(bar_x0, y, bar_x1, y, fill="gray70")

for i in range(NUM_LEVELS):
    y_center = bar_y1 - (i + 0.5) * level_height
    canvas.create_text(bar_x1 + 15, y_center, text=str(i))

# --------------------------
# Helper functions
# --------------------------
def workspace_to_canvas(x, y):
    """
    Convert normalized robot coords (0-1)
    to canvas pixels.
    """
    x_norm = x / WORKSPACE_WIDTH
    y_norm = y / WORKSPACE_HEIGHT
    cx = rect_x0 + x_norm * RECT_W
    cy = rect_y1 - y_norm * RECT_H
    return cx, cy


def move_marker(x, y):
    cx, cy = workspace_to_canvas(x, y)
    r = 5
    canvas.coords(robot_marker, cx-r, cy-r, cx+r, cy+r)

    half = EE_SIZE / 2
    canvas.coords(
        ee_square,
        cx-half, cy-half,
        cx+half, cy+half
    )

    tcp_x = x + OFFSET_X
    tcp_y = y + OFFSET_Y

    cx_tcp, cy_tcp = workspace_to_canvas(tcp_x, tcp_y)

    canvas.coords(robot_marker, cx_tcp-r, cy_tcp-r, cx_tcp+r, cy_tcp+r)

    canvas.coords(
        ee_center_line1,
        cx-EE_CENTER_SIZE, cy-EE_CENTER_SIZE,
        cx+EE_CENTER_SIZE, cy+EE_CENTER_SIZE
    )

    canvas.coords(
        ee_center_line2,
        cx-EE_CENTER_SIZE, cy+EE_CENTER_SIZE,
        cx+EE_CENTER_SIZE, cy-EE_CENTER_SIZE
    )

    canvas.tag_raise(ee_square)
    canvas.tag_raise(ee_center_line1)
    canvas.tag_raise(ee_center_line2)
    canvas.tag_raise(robot_marker)


def animate_robot_move(x0, y0, x1, y1, steps=40):
    """
    Fake animation so users SEE the movement.
    """
    for i in range(steps + 1):
        t = i / steps
        x = x0 + (x1-x0)*t
        y = y0 + (y1-y0)*t

        root.after_idle(move_marker, x, y)
        root.update()
        root.after(15)   # controls speed

def set_intensity(level):
    """
    level must be 0 to 5
    """
    snapped_y = bar_y0 + (NUM_LEVELS-1-level) * level_height
    canvas.coords(bar_fill, bar_x0, snapped_y, bar_x1, bar_y1)

# --------------------------
# Trial runner
# --------------------------
def run_demo_trial():
    global trial

    if trial > TOTAL_TRIALS:
        trial_label.config(text="Training Complete")
        return

    trial_label.config(text=f"Demo Trial {trial}/{TOTAL_TRIALS}")

    xi, yi, _ = points[trial-1]
    xf, yf, intensity = points[trial]

    set_intensity(0)  # clear bar immediately

    START_PAUSE = 2000          # see starting point
    PRE_STRETCH_DELAY = 1000    # pause after motion
    POST_STRETCH_VIEW = 2000   # see intensity

    # Show starting position
    move_marker(xi, yi)

    # Animate after a short pause
    def do_animation():
        animate_robot_move(xi, yi, xf, yf)

        # after animation finishes -> wait -> then stretch
        root.after(PRE_STRETCH_DELAY, start_robot)

    root.after(START_PAUSE, do_animation)

    # Robot runs in background
    def start_robot():

        def robot_thread():
            run_trial.run_trial(xi, yi, xf, yf, intensity)

            # update GUI safely from main thread
            root.after(0, lambda: set_intensity(int(intensity)))

            # auto advance
            root.after(POST_STRETCH_VIEW, next_trial)

        threading.Thread(target=robot_thread, daemon=True).start()

def next_trial():
    global trial
    trial += 1
    run_demo_trial()

def main():
    run_demo_trial()
    root.mainloop()

if __name__ == "__main__":
    main()