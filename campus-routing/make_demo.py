# Makes the three demo videos:
#   demo_closure.mp4   walkway closes mid route, D* Lite repairs the route
#   demo_crowd.mp4     a crowd makes a path slow, route goes around it
#   demo_stops.mp4     trip with stops, coffee then printing then class
#
# python3 make_demo.py     -> writes all three to results/ (gif without ffmpeg)

import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

import campus_nav as cn
import trip
from algorithm import DStarLite

HOLD = 9    # extra frames to sit on the interesting moment
TWEEN = 2   # in between frames while walking an edge so the agent glides
FPS = 3

# every demo assumes you leave at 9:00
LEAVE = datetime.datetime(2000, 1, 1, 9, 0)


def attime(seconds):
    """9:00 plus this many seconds, as a clock time."""
    return (LEAVE + datetime.timedelta(seconds=seconds)).strftime("%-I:%M")


def path_meters(path):
    """How many meters of walking are left on this path."""
    import math
    return sum(math.dist(cn.COORDS[path[i]], cn.COORDS[path[i + 1]])
               for i in range(len(path) - 1))


def stats(elapsed, path, extra_seconds=0):
    """The live corner text: clock, eta and distance left."""
    eta = attime(elapsed + cn.path_cost(path) + extra_seconds)
    return (f"now {attime(elapsed)}   eta {eta}\n"
            f"{path_meters(path):.0f} m to go")

# only label the places people recognize, labeling them all is unreadable
LANDMARKS = ["speare_hall", "snell_library", "curry_student_center",
             "ruggles_station", "isec", "west_village_a_north",
             "matthews_arena", "international_village", "ell_hall"]


def frame(walked, path, status, event=None, oldpath=None, stops=(), clock="",
          agentxy=None, panelold=None):
    """One frame of a demo. event is (edge, label, color) for the moment
    something happens on the map. agentxy is where the agent is drawn while
    walking along an edge, otherwise it sits on the last walked node.
    panelold makes the side panel show the abandoned route crossed out."""
    return {"walked": list(walked), "path": list(path), "status": status,
            "event": event, "oldpath": oldpath, "stops": stops,
            "clock": clock, "agentxy": agentxy, "panelold": panelold}


def glide(frames, walked, path, status, event, elapsed, cost, stops=(),
          extra_seconds=0):
    """The in between frames while the agent walks one edge."""
    x1, y1 = cn.COORDS[walked[-1]]
    x2, y2 = cn.COORDS[path[1]]
    for i in range(1, TWEEN + 1):
        t = i / (TWEEN + 1)
        spot = (x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)
        frames.append(frame(walked, path, status, event, stops=stops,
                            clock=stats(elapsed + cost * t, path, extra_seconds),
                            agentxy=spot))


def closure_frames():
    """Walkway closes mid route, D* Lite repairs cheap."""
    start, goal = "speare_hall", "west_village_a_north"
    close_edge = ("snell_engineering", "shillman_hall")
    cn.reset_edges()
    frames = []
    walked = [start]
    elapsed = 0.0
    planner = DStarLite(start, goal)
    path, expanded = planner.plan()
    status = f"planning... searched {expanded} nodes"
    event = None

    while len(path) > 1:
        if len(walked) - 1 == 6:
            oldpath = list(path)
            old = cn.block_edge(*close_edge)
            planner.notify_edge_change(*close_edge, old)
            path, expanded = planner.plan()
            _, redo = cn.astar(walked[-1], goal)
            status = (f"walkway closed! repaired route with {expanded} nodes "
                      f"(full redo would be {redo})")
            event = (close_edge, "closed", "#d64545")
            # first half of the freeze shows the old route getting crossed
            # out in the list, then the repaired one takes its place
            for i in range(HOLD):
                frames.append(frame(walked, path, status, event, oldpath,
                                    clock=stats(elapsed, path),
                                    panelold=oldpath if i < HOLD // 2 else None))
        frames.append(frame(walked, path, status, event,
                            clock=stats(elapsed, path)))
        cost = cn.edge_cost(walked[-1], path[1])
        glide(frames, walked, path, status, event, elapsed, cost)
        elapsed += cost
        planner.step_to(path[1])
        walked.append(path[1])
        path = path[1:]
        if event:
            status = "walking the new route"
    frames.append(frame(walked, [goal], f"arrived at {attime(elapsed)}", event,
                        clock=stats(elapsed, [goal])))
    return "Walkway closes, D* Lite repairs the route", frames


def crowd_frames():
    """A path jams up between classes, route goes around it."""
    start, goal = "west_village_h", "ell_hall"
    crowd_edge = ("snell_library", "curry_student_center")
    cn.reset_edges()
    frames = []
    walked = [start]
    elapsed = 0.0
    planner = DStarLite(start, goal)
    path, expanded = planner.plan()
    status = f"planning... searched {expanded} nodes"
    event = None

    while len(path) > 1:
        if len(walked) - 1 == 2:
            oldpath = list(path)
            old = cn.update_edge_cost(*crowd_edge, cn.EDGES[crowd_edge[0]][crowd_edge[1]] * 6)
            planner.notify_edge_change(*crowd_edge, old)
            path, expanded = planner.plan()
            status = (f"crowd ahead, that path is 6x slower now! "
                      f"rerouted with {expanded} nodes")
            event = (crowd_edge, "crowded", "#dd6b20")
            # same trick as the closure demo, cross out then replace
            for i in range(HOLD):
                frames.append(frame(walked, path, status, event, oldpath,
                                    clock=stats(elapsed, path),
                                    panelold=oldpath if i < HOLD // 2 else None))
        frames.append(frame(walked, path, status, event,
                            clock=stats(elapsed, path)))
        cost = cn.edge_cost(walked[-1], path[1])
        glide(frames, walked, path, status, event, elapsed, cost)
        elapsed += cost
        planner.step_to(path[1])
        walked.append(path[1])
        path = path[1:]
        if event:
            status = "walking the new route"
    frames.append(frame(walked, [goal], f"arrived at {attime(elapsed)}", event,
                        clock=stats(elapsed, [goal])))
    return "A crowd slows a path, the route goes around", frames


def stops_frames():
    """Dorm to class with a coffee stop and a printing stop on the way."""
    points = ["speare_hall", "curry_student_center", "snell_library", "isec"]
    labels = {"curry_student_center": "coffee",
              "snell_library": "printing"}
    cn.reset_edges()
    full, expanded = trip.route_with_stops(points[0], points[1:-1], points[-1])
    frames = []
    walked = [points[0]]
    elapsed = 0.0
    path = list(full)
    stops = tuple(points[1:-1])
    left = list(stops)  # stops we haven't hit yet, they add time to the eta
    status = f"{len(stops)} stops on the way, leaving at 9:00"

    def stoptime():
        return len(left) * trip.MINUTES_PER_STOP * 60

    while len(path) > 1:
        frames.append(frame(walked, path, status, stops=stops,
                            clock=stats(elapsed, path, stoptime())))
        cost = cn.edge_cost(walked[-1], path[1])
        glide(frames, walked, path, status, None, elapsed, cost,
              stops=stops, extra_seconds=stoptime())
        elapsed += cost
        nxt = path[1]
        walked.append(nxt)
        path = path[1:]
        if nxt in labels:
            # sit at the stop and let the clock tick through it instead of
            # jumping 5 minutes at once. the eta holds steady because the
            # stop time was already budgeted in
            stop_secs = trip.MINUTES_PER_STOP * 60
            left.remove(nxt)
            for i in range(HOLD):
                spent = stop_secs * (i + 1) / HOLD
                mins_left = (stop_secs - spent) / 60
                pause = (f"at {nxt.replace('_', ' ')}, {labels[nxt]} "
                         f"({mins_left:.0f} min left)")
                frames.append(frame(walked, path, pause, stops=stops,
                                    clock=stats(elapsed + spent, path,
                                                stoptime() + stop_secs - spent)))
            elapsed += stop_secs
    frames.append(frame(walked, [points[-1]],
                        f"made it to class at {attime(elapsed)}", stops=stops,
                        clock=stats(elapsed, [points[-1]])))
    return "Trip with stops: coffee, printing, then class", frames


def draw(ax, panel, title, f):
    ax.clear()
    panel.clear()
    ax.set_facecolor("#f4f2ec")
    walked, path = f["walked"], f["path"]

    for a, b, kind in cn.EDGE_SPECS:
        x1, y1 = cn.COORDS[a]
        x2, y2 = cn.COORDS[b]
        if f["event"] and {a, b} == set(f["event"][0]):
            edge, label, color = f["event"]
            ax.plot([x1, x2], [y1, y2], color=color, linewidth=4, zorder=3)
            # push the label away from the edge and put it in a solid white
            # box so it never disappears into nodes or other labels
            ax.annotate(label, ((x1 + x2) / 2, (y1 + y2) / 2), color=color,
                        fontsize=13, fontweight="bold",
                        xytext=(18, 20), textcoords="offset points", zorder=9,
                        bbox=dict(facecolor="white", edgecolor=color,
                                  boxstyle="round,pad=0.35"))
        else:
            style = "--" if kind == "stairs" else "-"
            ax.plot([x1, x2], [y1, y2], style, color="#cfccc4",
                    linewidth=1.5, zorder=1)

    # the route we just gave up on, during the freeze frames
    if f["oldpath"]:
        ax.plot([cn.COORDS[n][0] for n in f["oldpath"]],
                [cn.COORDS[n][1] for n in f["oldpath"]],
                color="#b0aca2", linewidth=2.5, linestyle=":", zorder=3)

    # while walking an edge the agent is partway along it, so the trail and
    # the route ahead both meet at that spot instead of jumping node to node
    agent = f["agentxy"] if f["agentxy"] else cn.COORDS[walked[-1]]

    trail_x = [cn.COORDS[n][0] for n in walked] + [agent[0]]
    trail_y = [cn.COORDS[n][1] for n in walked] + [agent[1]]
    if len(trail_x) > 1:
        ax.plot(trail_x, trail_y, color="#9bb8d3", linewidth=3.5, zorder=2)
    if len(path) > 1:
        ahead_x = [agent[0]] + [cn.COORDS[n][0] for n in path[1:]]
        ahead_y = [agent[1]] + [cn.COORDS[n][1] for n in path[1:]]
        ax.plot(ahead_x, ahead_y, color="#2b6cb0", linewidth=4, zorder=4,
                solid_capstyle="round")

    for name, (x, y) in cn.COORDS.items():
        junction = name.startswith("jct_")
        ax.scatter(x, y, s=8 if junction else 30, color="#8a8578", zorder=2)
    for name in LANDMARKS:
        x, y = cn.COORDS[name]
        ax.annotate(name.replace("_", " "), (x, y), fontsize=7.5,
                    color="#5a564c", xytext=(5, 5),
                    textcoords="offset points", zorder=5)

    # stops get an orange square so you can spot them
    for name in f["stops"]:
        x, y = cn.COORDS[name]
        ax.scatter(x, y, s=120, marker="s", color="#dd6b20",
                   edgecolor="white", zorder=6)

    gx, gy = cn.COORDS[path[-1]]
    ax.scatter(gx, gy, s=260, marker="*", color="#d69e2e",
               edgecolor="#8a6d1d", zorder=6)
    ax.scatter(agent[0], agent[1], s=180, color="#2f855a", edgecolor="white",
               linewidth=2, zorder=7)

    box = dict(facecolor="#f4f2ec", edgecolor="none", alpha=0.85,
               boxstyle="round,pad=0.3")
    ax.set_title(title, fontsize=13)
    ax.text(0.01, 0.02, f["status"], transform=ax.transAxes, fontsize=11,
            color="#333333", bbox=box, zorder=8)
    # clock and distance in the corner, left at 9:00
    ax.text(0.99, 0.02, f["clock"], transform=ax.transAxes, fontsize=14,
            color="#222222", ha="right", va="bottom", family="monospace",
            fontweight="bold", bbox=box, zorder=8)
    ax.set_aspect("equal")
    ax.axis("off")

    # side panel, the route written out live
    panel.set_facecolor("#f4f2ec")
    panel.axis("off")
    lines = [("route", "#333333", "bold")]
    for n in walked[:-1]:
        lines.append(("  - " + n.replace("_", " "), "#a5a094", "normal"))
    lines.append(("  > " + walked[-1].replace("_", " "), "#2f855a", "bold"))
    if f["panelold"]:
        # the stops we're about to lose, crossed out
        for n in f["panelold"][1:]:
            lines.append(("  x " + n.replace("_", " "), "#d64545", "normal"))
        lines.append(("", "#333333", "normal"))
        lines.append((f'{f["event"][1]} ahead,', "#d64545", "bold"))
        lines.append(("rerouting...", "#d64545", "bold"))
    else:
        for n in path[1:]:
            mark = "  * " if n in f["stops"] else "    "
            color = "#dd6b20" if n in f["stops"] else "#2b6cb0"
            lines.append((mark + n.replace("_", " "), color, "normal"))
        if f["oldpath"]:
            lines.append(("", "#333333", "normal"))
            lines.append(("new route!", "#2b6cb0", "bold"))
    for i, (text, color, weight) in enumerate(lines):
        panel.text(0.02, 0.97 - i * 0.042, text, transform=panel.transAxes,
                   fontsize=9.5, color=color, fontweight=weight,
                   verticalalignment="top")


def save_demo(name, title, frames):
    fig, (ax, panel) = plt.subplots(
        1, 2, figsize=(15, 10), gridspec_kw={"width_ratios": [3.2, 1]})
    fig.set_facecolor("#f4f2ec")
    anim = FuncAnimation(fig, lambda i: draw(ax, panel, title, frames[i]),
                         frames=len(frames), interval=1000 // FPS)
    try:
        anim.save(f"results/{name}.mp4", writer=FFMpegWriter(fps=FPS))
        print(f"wrote results/{name}.mp4 ({len(frames)} frames)")
    except Exception:
        anim.save(f"results/{name}.gif", writer=PillowWriter(fps=FPS))
        print(f"wrote results/{name}.gif ({len(frames)} frames)")
    plt.close(fig)


if __name__ == "__main__":
    for name, builder in [("demo_closure", closure_frames),
                          ("demo_crowd", crowd_frames),
                          ("demo_stops", stops_frames)]:
        title, frames = builder()
        save_demo(name, title, frames)
