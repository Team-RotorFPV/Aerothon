#!/usr/bin/env python3
"""Standalone Mission 2 (SkyScan) simulator — a *watchable* working model.

Runs the SAME logic as the ROS 2 stack, with zero ROS dependency:
  * 2D lidar (ray-cast) feeding the SAME sector-min corridor-centering law
    as avoidance/velocity_controller.py  (vy = k*(right-left), front braking)
  * the SAME mission sequence as mission_bt:
      takeoff -> to-corridor -> corridor(avoid) -> zone -> lawnmower search
      -> match QR -> winch drop -> return(avoid) -> land
  * GPS "position setpoints" for the open legs (P-controller to a waypoint)
    and body-frame reactive velocity in the corridor -- the hybrid from §4.1.

Renders an animated GIF + a static overview PNG so you can see it fly.

Run:  python3 sim/mission_sim.py
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation, PillowWriter

# --------------------------------------------------------------------------- #
# World (top-down, metres). Matches the rulebook geometry.
# --------------------------------------------------------------------------- #
START = np.array([2.0, 0.0])
CORRIDOR_X0, CORRIDOR_X1 = 5.0, 15.0     # 10 m corridor
HALF_W = 1.75                            # 3.5 m wide
ZONE = (15.0, 55.0, -15.0, 15.0)         # x0,x1,y0,y1 : 40 x 30 m delivery zone

# Corridor walls as segments (x1,y1,x2,y2)
WALLS = [
    (CORRIDOR_X0, HALF_W, CORRIDOR_X1, HALF_W),
    (CORRIDOR_X0, -HALF_W, CORRIDOR_X1, -HALF_W),
]
# Static obstacles = bumps protruding from alternating walls (cx,cy,r)
BUMPS = [(8.5, HALF_W, 0.9), (11.5, -HALF_W, 0.9)]

# QR codes in the delivery zone; TARGET is the matched one
QRS = [(25, 8), (35, -6), (45, 10), (30, -11), (48, -3)]
TARGET = np.array([35.0, -6.0])
REDZONE = (38.0, 46.0, 2.0, 10.0)        # x0,x1,y0,y1 keep-out

LIDAR_MAX = 12.0                         # RPLidar C1
DT = 0.05

# --------------------------------------------------------------------------- #
# Ray casting
# --------------------------------------------------------------------------- #
def _ray_segment(p, d, a, b):
    v1 = p - a
    v2 = b - a
    v3 = np.array([-d[1], d[0]])
    denom = v2 @ v3
    if abs(denom) < 1e-9:
        return None
    t = np.cross(v2, v1) / denom
    u = (v1 @ v3) / denom
    if t >= 0 and 0 <= u <= 1:
        return t
    return None

def _ray_circle(p, d, c, r):
    f = p - c
    b = 2 * (f @ d)
    cc = f @ f - r * r
    disc = b * b - 4 * cc
    if disc < 0:
        return None
    disc = math.sqrt(disc)
    for t in ((-b - disc) / 2, (-b + disc) / 2):
        if t >= 0:
            return t
    return None

def raycast(p, ang):
    d = np.array([math.cos(ang), math.sin(ang)])
    best = LIDAR_MAX
    for (x1, y1, x2, y2) in WALLS:
        t = _ray_segment(p, d, np.array([x1, y1]), np.array([x2, y2]))
        if t is not None:
            best = min(best, t)
    for (cx, cy, r) in BUMPS:
        t = _ray_circle(p, d, np.array([cx, cy]), r)
        if t is not None:
            best = min(best, t)
    return best

def sector_min(p, heading, rel_lo, rel_hi, step=4):
    best = LIDAR_MAX
    for deg in range(int(rel_lo), int(rel_hi) + 1, step):
        best = min(best, raycast(p, heading + math.radians(deg)))
    return best

# --------------------------------------------------------------------------- #
# Controllers (mirror the ROS nodes)
# --------------------------------------------------------------------------- #
def goto(pos, target, speed=2.0, kp=0.9):
    err = target - pos
    dist = np.linalg.norm(err)
    v = kp * err
    n = np.linalg.norm(v)
    if n > speed:
        v = v / n * speed
    return v, dist

def corridor_cmd(pos, heading, cruise=1.1, brake=2.5, stop=0.6, k_center=0.7):
    """Same law as avoidance/velocity_controller.py, in world frame."""
    front = sector_min(pos, heading, -25, 25)
    left = sector_min(pos, heading, 10, 110)
    right = sector_min(pos, heading, -110, -10)
    if front <= stop:
        vx = 0.0
    elif front >= brake:
        vx = cruise
    else:
        vx = cruise * (front - stop) / (brake - stop)
    vy_body = k_center * (right - left)               # +y_body = right
    vy_body = max(-0.7, min(0.7, vy_body))
    f = np.array([math.cos(heading), math.sin(heading)])
    r = np.array([math.sin(heading), -math.cos(heading)])
    return vx * f + vy_body * r, front, (right - left)

# --------------------------------------------------------------------------- #
# Lawnmower waypoints over the zone, skipping the red zone
# --------------------------------------------------------------------------- #
def in_red(x, y):
    x0, x1, y0, y1 = REDZONE
    return x0 - 1 <= x <= x1 + 1 and y0 - 1 <= y <= y1 + 1

def lawnmower():
    wps = []
    ys = np.arange(-12, 13, 6)
    xa, xb = 20.0, 52.0
    for i, y in enumerate(ys):
        xs = [xa, xb] if i % 2 == 0 else [xb, xa]
        for x in xs:
            if not in_red(x, y):
                wps.append(np.array([float(x), float(y)]))
    return wps

# --------------------------------------------------------------------------- #
# Simulate
# --------------------------------------------------------------------------- #
def simulate():
    pos = START.copy()
    vel = np.zeros(2)
    state = "TAKEOFF"
    heading = 0.0
    sweep = lawnmower()
    sw_i = 0
    drop_t = 0.0
    t = 0.0
    hist = []

    for step in range(6000):
        cmd = np.zeros(2)
        front = LIDAR_MAX

        if state == "TAKEOFF":
            cmd, _ = goto(pos, START, speed=0.5)
            if step > 20:
                state = "TO_CORRIDOR"
        elif state == "TO_CORRIDOR":
            cmd, dist = goto(pos, np.array([CORRIDOR_X0 + 0.5, 0.0]), speed=1.6)
            if dist < 0.4:
                state = "CORRIDOR"
        elif state == "CORRIDOR":
            heading = 0.0
            cmd, front, _ = corridor_cmd(pos, heading)
            if pos[0] > CORRIDOR_X1 + 0.3:
                state = "TO_ZONE"
        elif state == "TO_ZONE":
            cmd, dist = goto(pos, np.array([18.0, 0.0]), speed=1.6)
            if dist < 0.5:
                state = "SEARCH"
        elif state == "SEARCH":
            if np.linalg.norm(pos - TARGET) < 1.3:
                state = "DROP"
            else:
                if sw_i < len(sweep):
                    cmd, dist = goto(pos, sweep[sw_i], speed=2.2)
                    if dist < 0.6:
                        sw_i += 1
                else:
                    cmd, _ = goto(pos, TARGET, speed=2.0)
        elif state == "DROP":
            cmd, _ = goto(pos, TARGET, speed=0.6)
            drop_t += DT
            if drop_t > 2.0:
                state = "RETURN_TO_CORRIDOR"
        elif state == "RETURN_TO_CORRIDOR":
            cmd, dist = goto(pos, np.array([CORRIDOR_X1 - 0.5, 0.0]), speed=1.8)
            if dist < 0.4:
                state = "CORRIDOR_BACK"
        elif state == "CORRIDOR_BACK":
            heading = math.pi
            cmd, front, _ = corridor_cmd(pos, heading)
            if pos[0] < CORRIDOR_X0 - 0.3:
                state = "LAND"
        elif state == "LAND":
            cmd, dist = goto(pos, START, speed=1.4)
            if dist < 0.25:
                hist.append((pos.copy(), state, t, front))
                break

        # first-order velocity response + integrate
        vel += (cmd - vel) * 0.35
        pos = pos + vel * DT
        t += DT
        if step % 3 == 0:                # ~15 fps record
            hist.append((pos.copy(), state, t, front))
    return hist

# --------------------------------------------------------------------------- #
# Draw the static course
# --------------------------------------------------------------------------- #
def draw_course(ax):
    ax.set_facecolor("#0e1116")
    zx0, zx1, zy0, zy1 = ZONE
    ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0,
                           fill=True, color="#123a1f", ec="#2e7d46", lw=1.5, alpha=0.6))
    # corridor walls
    for (x1, y1, x2, y2) in WALLS:
        ax.plot([x1, x2], [y1, y2], color="#5aa9ff", lw=3)
    ax.plot([CORRIDOR_X0, CORRIDOR_X0], [-HALF_W, HALF_W], color="#3ad14e", lw=4)  # green banner
    for (cx, cy, r) in BUMPS:
        ax.add_patch(Circle((cx, cy), r, color="#8899aa"))
    # red zone
    rx0, rx1, ry0, ry1 = REDZONE
    ax.add_patch(Rectangle((rx0, ry0), rx1 - rx0, ry1 - ry0,
                           color="#c0392b", alpha=0.75))
    ax.text((rx0 + rx1) / 2, (ry0 + ry1) / 2, "RED", color="white",
            ha="center", va="center", fontsize=8, weight="bold")
    # QR markers
    for (x, y) in QRS:
        is_t = abs(x - TARGET[0]) < 0.1 and abs(y - TARGET[1]) < 0.1
        ax.add_patch(Rectangle((x - 0.7, y - 0.7), 1.4, 1.4,
                               color="#f1c40f" if is_t else "#7f8c9a",
                               ec="white", lw=1.2))
        if is_t:
            ax.text(x, y + 1.4, "TARGET", color="#f1c40f", ha="center", fontsize=7, weight="bold")
    ax.plot(*START, marker="o", color="#2ecc71", ms=10)
    ax.text(START[0], START[1] - 1.6, "START/LAND", color="#2ecc71", ha="center", fontsize=7)
    ax.set_xlim(-2, 58)
    ax.set_ylim(-17, 17)
    ax.set_aspect("equal")
    ax.set_title("AeroTHON 2026 — Mission 2 (SkyScan) autonomous run", color="white")
    ax.tick_params(colors="#889")
    for s in ax.spines.values():
        s.set_color("#334")

# --------------------------------------------------------------------------- #
def render(hist):
    xs = [h[0][0] for h in hist]
    ys = [h[0][1] for h in hist]

    # --- static overview ---
    fig, ax = plt.subplots(figsize=(11, 6.5))
    fig.patch.set_facecolor("#0e1116")
    draw_course(ax)
    ax.plot(xs, ys, color="#ff6b6b", lw=1.6, alpha=0.9)
    fig.savefig("sim/mission_sim.png", dpi=110, facecolor=fig.get_facecolor())
    print("wrote sim/mission_sim.png")

    # --- animation ---
    fig, ax = plt.subplots(figsize=(11, 6.5))
    fig.patch.set_facecolor("#0e1116")
    draw_course(ax)
    (trail,) = ax.plot([], [], color="#ff6b6b", lw=1.6, alpha=0.9)
    drone = ax.scatter([], [], s=90, color="#ffffff", ec="#ff6b6b", zorder=5)
    label = ax.text(-1, 15.4, "", color="white", fontsize=10, weight="bold")

    def update(i):
        trail.set_data(xs[:i + 1], ys[:i + 1])
        drone.set_offsets([xs[i], ys[i]])
        st, tt, fr = hist[i][1], hist[i][2], hist[i][3]
        label.set_text(f"t={tt:4.1f}s   state={st:<18s}  front={fr:4.1f} m")
        return trail, drone, label

    frames = range(0, len(hist), 1)
    anim = FuncAnimation(fig, update, frames=frames, blit=False, interval=60)
    anim.save("sim/mission_sim.gif", writer=PillowWriter(fps=18),
              savefig_kwargs={"facecolor": fig.get_facecolor()})
    print(f"wrote sim/mission_sim.gif ({len(hist)} frames)")


if __name__ == "__main__":
    h = simulate()
    print(f"simulated {len(h)} recorded frames; final state = {h[-1][1]}")
    render(h)
