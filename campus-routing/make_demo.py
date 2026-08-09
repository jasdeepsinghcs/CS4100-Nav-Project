# Makes the demo video. An agent walks a route, a walkway closes partway
# through, and D* Lite repairs the route. The side panel shows the route live,
# and the caption shows how few nodes the repair took vs redoing the search.
#
# python3 make_demo.py     -> results/demo.mp4 (or .gif without ffmpeg)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

import campus_nav as cn
from algorithm import DStarLite

START = "speare_hall"
GOAL = "west_village_a_north"
# close this edge once the agent has walked this many steps
CLOSE_AFTER = 6
CLOSE_EDGE = ("snell_engineering", "shillman_hall")

HOLD = 6   # extra frames to sit on the closure so people can see it
FPS = 2

# only label the places people recognize, labeling all 59 is unreadable
LANDMARKS = ["speare_hall", "snell_library", "curry_student_center",
             "ruggles_station", "isec", "west_village_a_north",
             "matthews_arena", "international_village"]


def build_frames():
    """Walk the route with D* Lite and record what to draw each step.
    Each frame is (walked, path ahead, status text, closed yet, old path)."""
    cn.reset_edges()
    frames = []
    walked = [START]
    planner = DStarLite(START, GOAL)
    path, expanded = planner.plan()
    status = f"planning... searched {expanded} nodes"

    while path and len(path) > 1:
        if len(walked) - 1 == CLOSE_AFTER:
            oldpath = list(path)  # remember the route we're about to lose
            old = cn.block_edge(*CLOSE_EDGE)
            planner.notify_edge_change(*CLOSE_EDGE, old)
            path, expanded = planner.plan()
            # what would a full redo have cost? run plain A* just to compare
            _, redo = cn.astar(walked[-1], GOAL)
            status = (f"walkway closed! repaired route with {expanded} nodes "
                      f"(full redo would be {redo})")
            for _ in range(HOLD):
                frames.append((list(walked), list(path), status, True, oldpath))
        frames.append((list(walked), list(path), status,
                       len(walked) - 1 >= CLOSE_AFTER, None))
        nxt = path[1]
        planner.step_to(nxt)
        walked.append(nxt)
        path = path[1:]
        if not status.startswith("walking"):
            status = f"walking, {cn.path_cost(path) / 60:.1f} min to go"

    frames.append((list(walked), [GOAL], "arrived", True, None))
    return frames


def draw_frame(ax, panel, walked, path, status, closed, oldpath):
    ax.clear()
    panel.clear()
    ax.set_facecolor("#f4f2ec")

    # all the walkways underneath
    for a, b, kind in cn.EDGE_SPECS:
        x1, y1 = cn.COORDS[a]
        x2, y2 = cn.COORDS[b]
        broken = closed and {a, b} == set(CLOSE_EDGE)
        if broken:
            ax.plot([x1, x2], [y1, y2], color="#d64545", linewidth=4, zorder=3)
            ax.annotate("closed", ((x1 + x2) / 2, (y1 + y2) / 2),
                        color="#d64545", fontsize=11, fontweight="bold",
                        xytext=(8, 8), textcoords="offset points", zorder=6)
        else:
            style = "--" if kind == "stairs" else "-"
            ax.plot([x1, x2], [y1, y2], style, color="#cfccc4",
                    linewidth=1.5, zorder=1)

    # the route we just abandoned, only during the closure freeze frames
    if oldpath:
        ax.plot([cn.COORDS[n][0] for n in oldpath],
                [cn.COORDS[n][1] for n in oldpath],
                color="#d64545", linewidth=2.5, linestyle=":", zorder=3)

    # where we've already walked, faded
    if len(walked) > 1:
        ax.plot([cn.COORDS[n][0] for n in walked],
                [cn.COORDS[n][1] for n in walked],
                color="#9bb8d3", linewidth=3.5, zorder=2)

    # the route still ahead
    if len(path) > 1:
        ax.plot([cn.COORDS[n][0] for n in path],
                [cn.COORDS[n][1] for n in path],
                color="#2b6cb0", linewidth=4, zorder=4,
                solid_capstyle="round")

    # buildings as dots, junctions smaller
    for name, (x, y) in cn.COORDS.items():
        junction = name.startswith("jct_")
        ax.scatter(x, y, s=8 if junction else 30, color="#8a8578", zorder=2)

    for name in LANDMARKS:
        x, y = cn.COORDS[name]
        ax.annotate(name.replace("_", " "), (x, y), fontsize=7.5,
                    color="#5a564c", xytext=(5, 5),
                    textcoords="offset points", zorder=5)

    # goal star and the agent on top of everything
    gx, gy = cn.COORDS[GOAL]
    ax.scatter(gx, gy, s=260, marker="*", color="#d69e2e",
               edgecolor="#8a6d1d", zorder=6)
    px, py = cn.COORDS[walked[-1]]
    ax.scatter(px, py, s=180, color="#2f855a", edgecolor="white",
               linewidth=2, zorder=7)

    ax.set_title("Campus navigation demo: Speare Hall to West Village A",
                 fontsize=13)
    ax.text(0.01, 0.02, status, transform=ax.transAxes, fontsize=11,
            color="#333333")
    ax.set_aspect("equal")
    ax.axis("off")

    # side panel, the route written out live. done places greyed, current
    # spot marked, the rest is what's still planned
    panel.set_facecolor("#f4f2ec")
    panel.axis("off")
    lines = [("route", "#333333", "bold")]
    for n in walked[:-1]:
        lines.append(("  - " + n.replace("_", " "), "#a5a094", "normal"))
    lines.append(("  > " + walked[-1].replace("_", " "), "#2f855a", "bold"))
    for n in path[1:]:
        lines.append(("    " + n.replace("_", " "), "#2b6cb0", "normal"))
    if oldpath:
        lines.append(("", "#333333", "normal"))
        lines.append(("rerouted around the", "#d64545", "bold"))
        lines.append(("closed walkway", "#d64545", "bold"))
    for i, (text, color, weight) in enumerate(lines):
        panel.text(0.02, 0.97 - i * 0.045, text, transform=panel.transAxes,
                   fontsize=9.5, color=color, fontweight=weight,
                   verticalalignment="top")


def main():
    frames = build_frames()
    fig, (ax, panel) = plt.subplots(
        1, 2, figsize=(15, 10), gridspec_kw={"width_ratios": [3.2, 1]})
    fig.set_facecolor("#f4f2ec")

    def draw(i):
        draw_frame(ax, panel, *frames[i])

    anim = FuncAnimation(fig, draw, frames=len(frames), interval=1000 // FPS)

    try:
        anim.save("results/demo.mp4", writer=FFMpegWriter(fps=FPS))
        print(f"wrote results/demo.mp4 ({len(frames)} frames)")
    except Exception:
        anim.save("results/demo.gif", writer=PillowWriter(fps=FPS))
        print(f"wrote results/demo.gif ({len(frames)} frames)")


if __name__ == "__main__":
    main()
