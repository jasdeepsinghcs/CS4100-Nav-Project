# Makes the demo video. Walks an agent along a route, closes a path in front
# of it partway through, and shows it rerouting.
#
# python3 make_demo.py     -> writes results/demo.mp4 (or .gif if no ffmpeg)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

import campus_nav as cn

START = "speare_hall"
GOAL = "west_village_a_north"
# close this path once the agent has walked this many steps
CLOSE_AFTER = 6
CLOSE_EDGE = ("snell_engineering", "shillman_hall")

HOLD = 8  # frames to pause on the moment the path closes


def build_frames():
    """Walk the route and record what to draw at each step."""
    cn.reset_edges()
    frames = []
    pos = START
    path, expanded = cn.astar(pos, GOAL)
    walked = 0

    while path and len(path) > 1:
        if walked == CLOSE_AFTER:
            cn.block_edge(*CLOSE_EDGE)
            path, expanded = cn.astar(pos, GOAL)
            # sit on this moment so people can see the route change
            for _ in range(HOLD):
                frames.append((pos, list(path), expanded, True))
        frames.append((pos, list(path), expanded, walked >= CLOSE_AFTER))
        pos = path[1]
        path = path[1:]
        walked += 1

    frames.append((GOAL, [GOAL], expanded, True))
    return frames


def main():
    frames = build_frames()
    fig, ax = plt.subplots(figsize=(11, 9))

    def draw(i):
        pos, path, expanded, closed = frames[i]
        ax.clear()

        # every walkway
        for a, b, kind in cn.EDGE_SPECS:
            x1, y1 = cn.COORDS[a]
            x2, y2 = cn.COORDS[b]
            broken = closed and {a, b} == set(CLOSE_EDGE)
            ax.plot([x1, x2], [y1, y2],
                    color="red" if broken else "0.8",
                    linewidth=3 if broken else 1,
                    linestyle="--" if kind == "stairs" and not broken else "-",
                    zorder=2 if broken else 1)

        # the route still ahead of us
        if len(path) > 1:
            ax.plot([cn.COORDS[n][0] for n in path],
                    [cn.COORDS[n][1] for n in path],
                    color="tab:blue", linewidth=3, zorder=3)

        for name, (x, y) in cn.COORDS.items():
            ax.scatter(x, y, s=10 if name.startswith("jct_") else 25,
                       color="0.4", zorder=4)

        gx, gy = cn.COORDS[GOAL]
        ax.scatter(gx, gy, s=200, marker="*", color="tab:red", zorder=5)
        px, py = cn.COORDS[pos]
        ax.scatter(px, py, s=160, color="tab:green", zorder=6)
        ax.annotate(pos.replace("_", " "), (px, py), fontsize=9,
                    xytext=(6, 6), textcoords="offset points", zorder=6)

        left = cn.path_cost(path) / 60
        note = "path closed, rerouting" if closed else "walking"
        ax.set_title(f"{note}   |   {left:.1f} min to go   |   "
                     f"{expanded} nodes expanded")
        ax.set_xlabel("meters east")
        ax.set_ylabel("meters north")
        ax.set_aspect("equal")

    anim = FuncAnimation(fig, draw, frames=len(frames), interval=600)

    try:
        anim.save("results/demo.mp4", writer=FFMpegWriter(fps=2))
        print(f"wrote results/demo.mp4 ({len(frames)} frames)")
    except Exception:
        # no ffmpeg installed, gif works everywhere
        anim.save("results/demo.gif", writer=PillowWriter(fps=2))
        print(f"wrote results/demo.gif ({len(frames)} frames)")


if __name__ == "__main__":
    main()
