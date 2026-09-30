"""Step 9: scan -> map -> A* -> follow a few steps -> repeat, reacting to stop signs and people.

Usage: python3 main.py <cm_right> <cm_forward>      e.g.  python3 main.py 50 150
"""
import sys
import time
from picarx import Picarx
from mapping import GridMap, CELL_CM, GRID_W
from planner import astar, MOVES
from detector import Detector

# ---- calibrate these on your floor ----
SPEED = 40               # motor power used for all moves
CM_PER_SEC = 20.0        # how far the car travels in 1 s at SPEED (measure with a tape)
TURN_ANGLE = 30          # steering angle for turns (positive = right on PiCar-X)
TURN_90_SEC = 1.6        # time at TURN_ANGLE + SPEED to turn ~90 degrees
TURN_RADIUS_CM = 20      # how far the car shifts forward/sideways during a 90 degree turn
TURN_CELLS = round(TURN_RADIUS_CM / CELL_CM)  # how many cells early to start a turn
GOAL_TOL_CELLS = 1       # "close enough" to the goal, in cells
USE_PAN_SCAN = False     # True if the ultrasonic sensor sits on the camera pan servo
PAN_SIGN = 1             # flip to -1 if obstacles show up mirrored on the map
SCAN_ANGLES = range(-60, 61, 10)
STEPS_BEFORE_RESCAN = 3  # straight moves to take before rescanning
SAFETY_CM = 20           # ultrasonic override: stop and back up if something is this close
CHECK_EVERY_SEC = 0.05   # how often the ultrasonic is checked while driving
BACKUP_CM = 20           # how far to reverse when blocked, to make room for turning
STOP_SIGN_WAIT = 3       # seconds to wait at a stop sign
STOP_SIGN_COOLDOWN = 8   # seconds before the same stop sign can stop us again
LEFT_TRIM = 1.0          # power multipliers for each rear motor. Lower the stronger side
RIGHT_TRIM = 0.60        # until the car drives straight (right is ~20% faster: 1 / 1.2)

# Car starts at the bottom-middle of the map, facing forward
x_cm = GRID_W // 2 * CELL_CM + CELL_CM / 2
y_cm = CELL_CM / 2
heading = 0
last_stop_time = 0.0
grid = GridMap()


def scan(px):
    """Return [(angle, distance_cm), ...] from the ultrasonic sensor."""
    if not USE_PAN_SCAN:
        return [(0, px.get_distance())]
    readings = []
    for a in SCAN_ANGLES:
        px.set_cam_pan_angle(a)
        time.sleep(0.15)
        readings.append((PAN_SIGN * a, px.get_distance()))
    px.set_cam_pan_angle(0)
    time.sleep(0.2)
    return readings


def check_camera(px, det):
    """Stop for stop signs, wait while a person is in view."""
    global last_stop_time
    labels = det.detect()
    if "person" in labels:
        px.stop()
        print("Person detected, waiting...")
        while "person" in det.detect():
            time.sleep(0.3)
        print("Person gone, continuing")
    if "stop sign" in labels and time.time() - last_stop_time > STOP_SIGN_COOLDOWN:
        px.stop()
        print("Stop sign, stopping")
        time.sleep(STOP_SIGN_WAIT)
        last_stop_time = time.time()


def motors(px, speed, steer=0):
    """Like px.forward(), but with a separate trim for each side. Negative speed = backward.
    Slows the inside rear wheel on turns, the same way the picarx library does."""
    scale = (100 - abs(steer)) / 100
    left = speed * (scale if steer < 0 else 1) * LEFT_TRIM
    right = speed * (scale if steer > 0 else 1) * RIGHT_TRIM
    px.set_motor_speed(1, left)     # motor 1 = left rear
    px.set_motor_speed(2, -right)   # motor 2 = right rear (mounted mirrored)


def drive(px, seconds, steer=0, turn_dir=0):
    """Drive forward for `seconds`, checking the ultrasonic every CHECK_EVERY_SEC.
    Stops early if something is too close, and puts that obstacle on the map so the
    next plan goes around it. Returns the fraction of the move that was completed
    (1.0 = finished, less than 1 = stopped early). turn_dir: +1/-1 if turning, else 0."""
    steps = max(1, round(seconds / CHECK_EVERY_SEC))
    px.set_dir_servo_angle(steer)
    motors(px, SPEED, steer)
    for i in range(steps):
        d = px.get_distance()
        if 0 < d < SAFETY_CM:
            px.stop()
            px.set_dir_servo_angle(0)
            done = i / steps
            # Mark the obstacle. Mid-turn, the car faces partway between old and new heading.
            grid.add_scan((x_cm, y_cm, heading + 90 * turn_dir * done), [(0, d)])
            return done
        time.sleep(seconds / steps)
    px.stop()
    px.set_dir_servo_angle(0)
    return 1.0


def turn(px, direction):
    """direction = +1 right, -1 left. Turns ~90 degrees and updates the pose.
    Returns False if an obstacle interrupted the turn."""
    global x_cm, y_cm, heading
    done = drive(px, TURN_90_SEC, direction * TURN_ANGLE, direction)
    if done < 1:
        print("Too close during turn, undoing it and backing up")
        # Reverse along the same arc so the car ends up where the turn started
        px.set_dir_servo_angle(direction * TURN_ANGLE)
        motors(px, -SPEED, direction * TURN_ANGLE)
        time.sleep(done * TURN_90_SEC)
        px.stop()
        px.set_dir_servo_angle(0)
        back_up(px)
        return False
    old = MOVES[heading // 90]
    heading = (heading + 90 * direction) % 360
    new = MOVES[heading // 90]
    # A 90 degree arc moves the car one radius along the old heading and one along the new
    x_cm += TURN_RADIUS_CM * (old[0] + new[0])
    y_cm += TURN_RADIUS_CM * (old[1] + new[1])
    return True


def turn_to(px, target_heading):
    """Turn to face target_heading. Returns False if an obstacle interrupted it."""
    diff = (target_heading - heading) % 360
    if diff == 90:
        return turn(px, 1)
    if diff == 270:
        return turn(px, -1)
    if diff == 180:
        return turn(px, 1) and turn(px, 1)
    return True


def back_up(px):
    """Reverse straight back BACKUP_CM so the next turn doesn't hit the obstacle."""
    global x_cm, y_cm
    px.stop()
    px.set_dir_servo_angle(0)
    time.sleep(0.3)          # give the steering servo time to straighten before reversing
    motors(px, -SPEED)
    time.sleep(BACKUP_CM / CM_PER_SEC)
    px.stop()
    dx, dy = MOVES[heading // 90]
    x_cm -= dx * BACKUP_CM
    y_cm -= dy * BACKUP_CM


def forward_one_cell(px):
    """Move one cell forward. Returns False if something got too close on the way."""
    global x_cm, y_cm
    done = drive(px, CELL_CM / CM_PER_SEC)
    dx, dy = MOVES[heading // 90]
    x_cm += dx * CELL_CM * done  # count only the part of the move that happened
    y_cm += dy * CELL_CM * done
    if done < 1:
        print("Too close, backing up and rescanning")
        back_up(px)
        return False
    return True


def main():
    goal_right, goal_forward = float(sys.argv[1]), float(sys.argv[2])
    goal = grid.to_cell(x_cm + goal_right, y_cm + goal_forward)

    px = Picarx()
    det = Detector()
    px.set_dir_servo_angle(0)
    try:
        while True:
            car = grid.to_cell(x_cm, y_cm)
            # Position is only an estimate, so being within 1 cell counts as arriving
            if abs(car[0] - goal[0]) <= GOAL_TOL_CELLS and abs(car[1] - goal[1]) <= GOAL_TOL_CELLS:
                print("Reached goal!")
                break

            grid.add_scan((x_cm, y_cm, heading), scan(px))
            plan_grid = grid.padded()
            # Padding must never swallow the goal. Right around the goal, only real
            # detections (#) block, and the goal square itself is always allowed.
            gx, gy = goal
            r = GOAL_TOL_CELLS
            ys, xs = slice(max(0, gy - r), gy + r + 1), slice(max(0, gx - r), gx + r + 1)
            plan_grid[ys, xs] = grid.raw[ys, xs]
            plan_grid[gy, gx] = 0
            path = astar(plan_grid, car, goal, heading // 90)
            grid.show(car, goal, path)
            if path is None:
                print("No path found")
                break

            # Direction of each step along the path (0 fwd, 1 right, 2 back, 3 left)
            dirs = [MOVES.index((b[0] - a[0], b[1] - a[1])) for a, b in zip(path, path[1:])]
            straight = 0
            for i in range(len(dirs)):
                check_camera(px, det)
                # Where is the next turn on the path?
                t = next((k for k in range(i, len(dirs)) if dirs[k] != heading // 90), None)
                if t is not None and t - i <= TURN_CELLS:
                    # A turn swings the car about TURN_CELLS forward, so start it early.
                    # That way the car comes out of the turn on the row/column A* planned.
                    turn_to(px, dirs[t] * 90)
                    break  # rescan from the new direction
                if not forward_one_cell(px):
                    break
                straight += 1
                if straight >= STEPS_BEFORE_RESCAN:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        px.stop()
        det.close()


if __name__ == "__main__":
    main()
