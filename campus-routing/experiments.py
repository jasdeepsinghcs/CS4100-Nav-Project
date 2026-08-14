"""Walks an agent along a route, breaks the map partway through, and records
what each planner had to do to recover.

Naive baseline reruns the whole A* search from wherever the agent is standing.
D* Lite repairs instead.
"""

import csv
import os
import time

import campus_nav as cn

# kriti's D* Lite lives in algorithm.py
from algorithm import DStarLite


# ============ event injection ============

def apply_event(event):
    """Break or slow down an edge. Returns the old cost so the planner can be
    told what changed."""
    a, b = event["edge"]
    if event["action"] == "block":
        return cn.block_edge(a, b)
    # crowding, the path still works it just gets slower
    return cn.update_edge_cost(a, b, cn.EDGES[a][b] * event["factor"])


# Speare to West Village A is the main test trip. It's 15 nodes right across
# campus, and every walkway on it has a detour if you close it, so no scenario
# can end with the agent stranded.
#
# step is how many nodes the agent has already walked when the event fires
SCENARIOS = [
    {
        "name": "control_no_change",
        "start": "speare_hall", "goal": "west_village_a_north",
        "events": [],
    },
    {
        "name": "block_near_start",
        "start": "speare_hall", "goal": "west_village_a_north",
        "events": [
            {"step": 1, "action": "block", "edge": ("holmes_hall", "kariotis_hall")},
        ],
    },
    {
        "name": "block_mid_route",
        "start": "speare_hall", "goal": "west_village_a_north",
        "events": [
            {"step": 7, "action": "block", "edge": ("snell_engineering", "shillman_hall")},
        ],
    },
    {
        "name": "block_near_goal",
        "start": "speare_hall", "goal": "west_village_a_north",
        "events": [
            {"step": 11, "action": "block", "edge": ("west_village_e", "west_village_c")},
        ],
    },
    {
        "name": "multiple_changes",
        "start": "speare_hall", "goal": "west_village_a_north",
        "events": [
            {"step": 1, "action": "block", "edge": ("holmes_hall", "kariotis_hall")},
            {"step": 5, "action": "crowd", "edge": ("centennial_common", "egan_center"), "factor": 4.0},
            {"step": 9, "action": "block", "edge": ("ryder_hall", "behrakis_center")},
        ],
    },
    {
        "name": "crowd_spike",
        "start": "west_village_h", "goal": "ell_hall",
        "events": [
            {"step": 2, "action": "crowd", "edge": ("snell_library", "curry_student_center"), "factor": 6.0},
        ],
    },
]


# ============ planners ============
# both wrappers look the same to the simulation so the loop below doesn't
# care which one it's driving

class NaivePlanner:
    """The baseline. Throws away everything and reruns A* on every change."""

    name = "naive_astar"

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

    def plan(self):
        return cn.astar(self.start, self.goal)

    def step_to(self, node):
        self.start = node

    def notify_edge_change(self, a, b, old_cost):
        pass  # nothing to tell it, it starts from scratch anyway


class DStarPlanner:
    """Thin wrapper over Kriti's class so it plugs into the same loop."""

    name = "dstar_lite"

    def __init__(self, start, goal):
        self.inner = DStarLite(start, goal)

    def plan(self):
        return self.inner.plan()

    def step_to(self, node):
        self.inner.step_to(node)

    def notify_edge_change(self, a, b, old_cost):
        self.inner.notify_edge_change(a, b, old_cost)


# ============ simulation ============

def simulate(scenario, planner_class, log_rows):
    """Walk from start to goal, firing events on the way. One log row per
    replan. Returns a summary dict."""
    cn.reset_edges()
    start, goal = scenario["start"], scenario["goal"]
    planner = planner_class(start, goal)

    total_expanded = 0
    total_ms = 0.0
    travel_seconds = 0.0
    replans = 0

    def timed_plan(trigger):
        nonlocal total_expanded, total_ms, replans
        t0 = time.perf_counter()
        path, expanded = planner.plan()
        ms = (time.perf_counter() - t0) * 1000

        # warn if the path doesn't start where the agent is or end at the goal
        if path and (path[0] != pos or path[-1] != goal):
            print(f"  WARNING {planner.name} bad path in {scenario['name']}: "
                  f"starts at {path[0]} (expected {pos}), "
                  f"ends at {path[-1]} (expected {goal})")

        total_expanded += expanded
        total_ms += ms
        replans += 1
        log_rows.append({
            "scenario": scenario["name"],
            "planner": planner.name,
            "replan": replans - 1,
            "trigger": trigger,
            "nodes_expanded": expanded,
            "millis": round(ms, 3),
            # what's LEFT from where the agent is standing, not the whole trip.
            # it shrinks as you walk, don't read it as the route getting faster.
            "remaining_seconds": round(cn.path_cost(path), 1) if path else "",
            "remaining_nodes": len(path),
        })
        return path

    pos = start
    path = timed_plan("initial")
    walked = 0
    stuck = False

    # cap the loop so a broken planner can't spin forever
    while path and len(path) > 1 and walked < 100:
        due = [e for e in scenario["events"] if e["step"] == walked]
        if due:
            for event in due:
                a, b = event["edge"]
                old = apply_event(event)
                planner.notify_edge_change(a, b, old)
            path = timed_plan(f"{due[0]['action']} {due[0]['edge'][0]}-{due[0]['edge'][1]}")
            if not path:
                stuck = True
                break

        nxt = path[1]
        travel_seconds += cn.edge_cost(pos, nxt)
        planner.step_to(nxt)
        pos = nxt
        walked += 1
        path = path[1:]

    return {
        "scenario": scenario["name"],
        "planner": planner.name,
        "replans": replans,
        "total_expanded": total_expanded,
        "total_millis": round(total_ms, 3),
        "steps_walked": walked,
        "travel_minutes": round(travel_seconds / 60, 2),
        "arrived": not stuck and walked < 100,
    }


# ============ runner ============

def main():
    planners = [NaivePlanner, DStarPlanner]

    log_rows = []
    summaries = []
    for scenario in SCENARIOS:
        for planner_class in planners:
            summaries.append(simulate(scenario, planner_class, log_rows))

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, "replans.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
        writer.writeheader()
        writer.writerows(log_rows)

    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    header = f"{'scenario':<20} {'planner':<12} {'replans':>7} {'expanded':>9} {'ms':>8} {'walk min':>9}  arrived"
    print(header)
    print("-" * len(header))
    for s in summaries:
        print(f"{s['scenario']:<20} {s['planner']:<12} {s['replans']:>7} "
              f"{s['total_expanded']:>9} {s['total_millis']:>8.2f} "
              f"{s['travel_minutes']:>9.2f}  {s['arrived']}")

    print(f"\nwrote {len(log_rows)} replan rows to results/")


if __name__ == "__main__":
    main()
