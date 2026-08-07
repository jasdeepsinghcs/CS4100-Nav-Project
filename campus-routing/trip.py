# Routes with stops along the way, plus working out when to leave.
# Stops get visited in the order you give them, we don't reorder them.

import datetime

import campus_nav as cn

MINUTES_PER_STOP = 5  # time spent grabbing coffee, printing, etc


def route_with_stops(start, stops, goal):
    # run A* for each leg and stick the results together
    points = [start] + list(stops) + [goal]
    path = []
    expanded = 0

    for i in range(len(points) - 1):
        leg, n = cn.astar(points[i], points[i + 1])
        expanded += n
        if not leg:
            return [], expanded
        # skip the first node after leg 1, it's the same as the last one
        path += leg if i == 0 else leg[1:]

    return path, expanded


def trip_seconds(path, stops):
    # walking time plus the time you spend at each stop
    return cn.path_cost(path) + len(stops) * MINUTES_PER_STOP * 60


def leave_by(arrive_at, seconds):
    # arrive_at is like "10:00"
    h, m = arrive_at.split(":")
    arrive = datetime.datetime(2000, 1, 1, int(h), int(m))
    return (arrive - datetime.timedelta(seconds=seconds)).strftime("%H:%M")


def show_trip(start, stops, goal, arrive_at=None):
    path, expanded = route_with_stops(start, stops, goal)
    if not path:
        print("no route, something on the way is blocked")
        return

    points = [start] + list(stops) + [goal]
    for i in range(len(points) - 1):
        leg, _ = cn.astar(points[i], points[i + 1])
        print(f"  walk {cn.path_cost(leg) / 60:4.1f} min   {points[i]} -> {points[i + 1]}")
        if i < len(points) - 2:
            print(f"  stop {MINUTES_PER_STOP:4} min   at {points[i + 1]}")

    total = trip_seconds(path, stops)
    print(f"  total {total / 60:.1f} min, {expanded} nodes expanded")
    if arrive_at:
        print(f"  leave at {leave_by(arrive_at, total)} to get there by {arrive_at}")


if __name__ == "__main__":
    START, STOPS, GOAL = "speare_hall", ["curry_student_center", "snell_library"], "isec"

    print("dorm, coffee at curry, print at snell, then class in ISEC:")
    show_trip(START, STOPS, GOAL, arrive_at="10:00")

    print()
    print("same trip if you can't do stairs (the bridge to ISEC is steps):")
    cn.AVOID_STAIRS = True
    show_trip(START, STOPS, GOAL, arrive_at="10:00")
    cn.AVOID_STAIRS = False

    print()
    print("same trip again but the bridge is closed for construction:")
    cn.block_edge("isec_bridge", "isec")
    show_trip(START, STOPS, GOAL, arrive_at="10:00")
    cn.reset_edges()
