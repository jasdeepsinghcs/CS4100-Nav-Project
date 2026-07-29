# I own the graph section, Kriti owns the algorithms section


# node ids are strings, coords are meters, edge costs are SECONDS.
# heuristic has to be seconds too or it stops being admissible and both
# algorithms give wrong answers without erroring.

import math


# ============ GRAPH (A) ============

# node -> (x, y) in meters. flat made up grid, origin wherever.
# only used by the heuristic for straight line distance.
COORDS = {
    "snell": (0, 0),
    "centennial": (45, 30),
    "curry": (90, 20),
}

# node -> {neighbor: seconds}
# list every edge BOTH ways or the graph goes one directional
EDGES = {
    "snell": {"centennial": 40},
    "centennial": {"snell": 40, "curry": 35},
    "curry": {"centennial": 35},
}

WALK_SPEED = 1.4  # m/s, used to turn meters into seconds


def get_neighbors(node):
    """Nodes you can walk to directly from here."""
    return list(EDGES[node].keys())


def edge_cost(a, b):
    """Seconds to walk a -> b. inf if blocked or no edge exists."""
    return EDGES[a].get(b, float("inf"))


def heuristic(a, b):
    """Estimated seconds a -> b, straight line.
    Admissible because no real path is shorter than a straight line,
    which is what makes A* optimal."""

    x1, y1 = COORDS[a]
    x2, y2 = COORDS[b]
    return math.hypot(x2 - x1, y2 - y1) / WALK_SPEED


def block_edge(a, b):
    """Obstacle appears. Sets edge to inf both ways, RETURNS THE OLD COST.

    D* Lite needs the old cost to work out which nodes went inconsistent,
    so don't make this return None."""
    old = EDGES[a][b]
    EDGES[a][b] = float("inf")
    EDGES[b][a] = float("inf")
    return old


def plot_graph(path=None, blocked=None):
    """Matplotlib. Nodes at their COORDS, lines for edges, path highlighted,
    blocked edges in red. Just needs to be readable for the report."""
    pass


# ============ ALGORITHMS (B) ============

def astar(start, goal):
    """Plain A*, should be the baseline.
    Returns (path, nodes_expanded). path is a list of node ids including
    start and goal, or [] if no route. nodes_expanded is how many nodes we
    popped off the queue - that's the number we plot."""
    pass


class DStarLite:
    """Replans cheaply when the map changes while you're already walking.

    Each node has two values instead of one: g (best known cost to goal) and
    rhs (one step lookahead). If they match the node is consistent. When an
    edge changes we mark the affected nodes inconsistent and repair outward
    from there, instead of redoing the whole search like A* would.

    Searches backward from the goal, because as you walk your start keeps
    moving but the goal doesn't, so anchoring at the goal means walking
    doesn't invalidate the search.

    Usage:
        p = DStarLite("snell", "curry")
        path, n = p.plan()             # full search, n is big
        p.step_to(path[1])             # walk a step
        old = block_edge("centennial", "curry")
        p.notify_edge_change("centennial", "curry", old)
        path, n = p.plan()             # repair, n should be tiny
    """

    def __init__(self, start, goal):
        """g and rhs dicts (all inf except rhs[goal] = 0), priority queue with
        goal pushed on, km = 0.

        km is an offset on the queue keys so that when start moves we shift
        everything by a constant instead of recomputing every entry.
        """
        pass

    def plan(self):
        """Compute or repair the path. Returns (path, nodes_expanded).

        nodes_expanded is PER CALL not cumulative - the whole result is
        "first call 200, repair 12". Cumulative shows nothing.
        """
        pass

    def step_to(self, node):
        """Walk one node forward. Updates start and bumps km. No searching."""
        pass

    def notify_edge_change(self, a, b, old_cost):
        """Call after block_edge. Recompute rhs for the two nodes on the
        changed edge and requeue them if inconsistent. Repair happens on the
        next plan()."""
        pass