# I own the graph section, Kriti owns the algorithms section


# node ids are strings, coords are meters, edge costs are SECONDS.
# heuristic has to be seconds too or it stops being admissible and both
# algorithms give wrong answers without erroring.

import copy
import heapq
import math


# ============ GRAPH (A) ============

# northeastern buildings, approximate lat/lon pulled off the campus map.
# these only feed the heuristic so being a few meters off is fine.
LATLON = {
    # academic buildings, north half of campus
    "ell_hall": (42.33985098651482, -71.08816141201183),
    "hayden_hall": (42.33929437783783, -71.0885281858779),
    "richards_hall": (42.33980780153514, -71.08874565356842),
    "dodge_hall": (42.34013168816623, -71.08777840919971),
    "churchill_hall": (42.33873631376955, -71.0888352876459),
    "cargill_hall": (42.338963567691565, -71.09159651019637),
    "kariotis_hall": (42.33863294608253, -71.09086658485944),
    "dockser_hall": (42.33861271603461, -71.09042072600592),
    "robinson_hall": (42.339267510336704, -71.08651456842905),
    "hurtig_hall": (42.33964405854107, -71.08620527027742),
    "meserve_hall": (42.33756967745806, -71.09086083764724),
    "lake_hall": (42.338283236362635, -71.0908414034151),
    "nightingale_hall": (42.33807847876291, -71.09011882961023),
    "holmes_hall": (42.33802927046173, -71.09076014985052),
    "knowles_center": (42.339230514545804, -71.09081772776169),

    # middle of campus
    "snell_library": (42.338571131764866, -71.08823515354734),
    "curry_student_center": (42.33902430573537, -71.08756297378758),
    "centennial_common": (42.33714475841921, -71.09042757290882),
    "krentzman_quad": (42.34020989844215, -71.08831935669879),
    "snell_engineering": (42.33827487272528, -71.08874314570049),
    "egan_center": (42.33780332837184, -71.08879196375477),
    "shillman_hall": (42.337492608006606, -71.09029968603201),
    "ryder_hall": (42.33648959178166, -71.090587383342),

    # west side toward huntington ave
    "forsyth_building": (42.3385576470295, -71.0898964609995),
    "mugar_life_sciences": (42.3397160579856, -71.08706299587634),
    "behrakis_center": (42.33665155610561, -71.0916480918257),
    "west_village_f": (42.33758975278314, -71.09118910523377),
    "west_village_h": (42.33849082138689, -71.09241886340898),
    "northeastern_tstop": (42.34002030748497, -71.08996131391557),

    # dorms and rec, north east corner
    "speare_hall": (42.340617402865625, -71.08979694016212),
    "stetson_west": (42.340617402865625, -71.08979694016212),
    "stetson_east": (42.34147578690803, -71.09001594732432),
    "marino_center": (42.3401360210806, -71.09039632817314),
    "cabot_center": (42.3392797914226, -71.08939180384694),
    "matthews_arena": (42.3409700579917, -71.08465750590888),
    "squashbusters": (42.33796544697437, -71.0859173355979),

    # south, including everything across the train tracks
    "international_village": (42.33547536135178, -71.08905893414617),
    "isec": (42.33759043091567, -71.08778368104026),
    "exp_building": (42.33701411029365, -71.08733057301396),
    "columbus_garage": (42.33811134294903, -71.0866975859222),
    "carter_playground": (42.33886021552859, -71.08502892728653),
    "renaissance_park": (42.33550992677049, -71.08827528382076),
    "ruggles_station": (42.33670399478578, -71.08909079369701),

    # walkway corners, not buildings. these are here so routes bend like real
    # sidewalks instead of cutting through the middle of buildings.

    # place between snell library and snell engineering, by the steps
    "jct_snell_quad": (42.33886320501888, -71.08816597818887),
    # huntington ave sidewalk outside the northeastern T stop
    "jct_huntington": (42.340104576362755, -71.08994242860005),
    # corners on the west village to IV walk, it goes down leon st then
    # along ruggles st instead of straight across
    "jct_leon_st_wv": (42.338479781142425, -71.09095393348582),
    "jct_ruggles_st": (42.33610244627593, -71.09116514717297),
    # corners on the matthews to squashbusters walk, out to mass ave then
    # turns onto columbus ave
    "jct_mass_ave": (42.342055796084814, -71.0841190988889),
    "jct_columbus_ave": (42.34057080704852, -71.08196247581918),
    # the ruggles upper busway
    "ruggles_busway": (42.33731354881061, -71.08891247273189),
    # main campus end of the bridge over the tracks, not ISEC itself
    "isec_bridge": (42.337591101995606, -71.08779049576968),
}

# anchor for the lat/lon -> meters conversion, roughly the middle of campus
_LAT0, _LON0 = 42.3390, -71.0890
_M_PER_LAT = 110540.0
_M_PER_LON = 111320.0 * math.cos(math.radians(_LAT0))

# node -> (x, y) in meters. flat made up grid, origin wherever.
# only used by the heuristic for straight line distance.
COORDS = {
    name: ((lon - _LON0) * _M_PER_LON, (lat - _LAT0) * _M_PER_LAT)
    for name, (lat, lon) in LATLON.items()
}

WALK_SPEED = 1.4  # m/s, used to turn meters into seconds

# how much slower each kind of path is than an open sidewalk.
# all of these have to be >= 1.0, otherwise a real path could beat the
# straight line and the heuristic stops being admissible.
KIND_SLOWDOWN = {
    "walk": 1.0,
    "busy": 1.25,    # hallways and doors that jam up between classes
    "ramp": 1.15,    # step free but takes the long way round
    "stairs": 1.35,  # slower, and unusable if AVOID_STAIRS is on
}

# every walkway on campus as (a, b, kind). written once, both directions
# get built below.
EDGE_SPECS = [
    # north quad
    ("lake_hall", "nightingale_hall", "walk"),
    ("lake_hall", "hurtig_hall", "walk"),
    ("nightingale_hall", "holmes_hall", "walk"),
    ("nightingale_hall", "meserve_hall", "walk"),
    ("meserve_hall", "hurtig_hall", "walk"),
    ("meserve_hall", "robinson_hall", "walk"),
    ("hurtig_hall", "hayden_hall", "walk"),
    ("robinson_hall", "ell_hall", "walk"),
    ("robinson_hall", "richards_hall", "walk"),
    ("hayden_hall", "ell_hall", "walk"),
    ("hayden_hall", "krentzman_quad", "walk"),
    ("ell_hall", "richards_hall", "busy"),
    ("ell_hall", "curry_student_center", "busy"),
    ("richards_hall", "curry_student_center", "walk"),
    ("richards_hall", "churchill_hall", "walk"),
    ("churchill_hall", "cargill_hall", "walk"),
    ("churchill_hall", "centennial_common", "stairs"),
    ("cargill_hall", "kariotis_hall", "walk"),
    ("kariotis_hall", "dockser_hall", "walk"),
    ("kariotis_hall", "holmes_hall", "walk"),
    ("dockser_hall", "knowles_center", "walk"),
    ("dockser_hall", "stetson_east", "walk"),
    ("knowles_center", "squashbusters", "walk"),
    ("knowles_center", "centennial_common", "walk"),
    ("holmes_hall", "speare_hall", "walk"),
    ("speare_hall", "stetson_west", "walk"),
    ("stetson_west", "stetson_east", "walk"),
    ("stetson_west", "marino_center", "walk"),
    ("marino_center", "stetson_east", "walk"),
    ("marino_center", "cabot_center", "walk"),
    ("cabot_center", "matthews_arena", "walk"),
    ("matthews_arena", "jct_mass_ave", "walk"),
    ("jct_mass_ave", "jct_columbus_ave", "walk"),
    ("jct_columbus_ave", "squashbusters", "walk"),

    # middle of campus
    ("curry_student_center", "centennial_common", "walk"),
    ("curry_student_center", "snell_library", "busy"),
    ("centennial_common", "snell_library", "walk"),
    ("centennial_common", "egan_center", "walk"),
    ("snell_library", "jct_snell_quad", "walk"),
    ("snell_library", "krentzman_quad", "walk"),
    ("snell_library", "forsyth_building", "walk"),
    ("snell_library", "shillman_hall", "walk"),
    ("jct_snell_quad", "egan_center", "walk"),
    ("jct_snell_quad", "snell_engineering", "stairs"),
    ("egan_center", "snell_engineering", "walk"),
    ("krentzman_quad", "dodge_hall", "walk"),
    ("dodge_hall", "hayden_hall", "walk"),
    ("dodge_hall", "northeastern_tstop", "walk"),
    ("shillman_hall", "snell_engineering", "walk"),
    ("shillman_hall", "ryder_hall", "walk"),
    ("shillman_hall", "international_village", "walk"),

    # west side
    ("northeastern_tstop", "jct_huntington", "walk"),
    ("jct_huntington", "behrakis_center", "walk"),
    ("jct_huntington", "forsyth_building", "walk"),
    ("forsyth_building", "mugar_life_sciences", "walk"),
    ("forsyth_building", "ryder_hall", "walk"),
    ("mugar_life_sciences", "behrakis_center", "walk"),
    ("west_village_f", "west_village_h", "stairs"),
    ("west_village_f", "ryder_hall", "walk"),
    ("west_village_h", "jct_leon_st_wv", "ramp"),
    ("jct_leon_st_wv", "jct_ruggles_st", "ramp"),
    ("jct_ruggles_st", "international_village", "ramp"),

    # south and across the tracks
    ("international_village", "ruggles_busway", "walk"),
    ("international_village", "ruggles_station", "walk"),
    ("ruggles_busway", "ruggles_station", "walk"),
    ("ruggles_busway", "isec_bridge", "walk"),
    ("ruggles_busway", "columbus_garage", "walk"),
    ("isec_bridge", "snell_engineering", "walk"),
    ("isec_bridge", "isec", "stairs"),
    ("columbus_garage", "isec", "ramp"),
    ("columbus_garage", "carter_playground", "walk"),
    ("columbus_garage", "renaissance_park", "walk"),
    ("isec", "exp_building", "walk"),
    ("exp_building", "carter_playground", "walk"),
    ("renaissance_park", "ruggles_station", "walk"),
]

# flip this on for the accessible routes feature. edge_cost then refuses
# to use any stairs edge, so A* routes around them on its own.
AVOID_STAIRS = False


def _seconds(a, b, kind):
    """Straight line meters between two nodes, converted to walking seconds."""
    x1, y1 = COORDS[a]
    x2, y2 = COORDS[b]
    return math.hypot(x2 - x1, y2 - y1) * KIND_SLOWDOWN[kind] / WALK_SPEED


def _build_edges():
    """Turn EDGE_SPECS into the both directions adjacency dict."""
    edges = {name: {} for name in COORDS}
    for a, b, kind in EDGE_SPECS:
        cost = _seconds(a, b, kind)
        edges[a][b] = cost
        edges[b][a] = cost
    return edges


# node -> {neighbor: seconds}
# list every edge BOTH ways or the graph goes one directional
EDGES = _build_edges()

# node pair -> kind, so we can tell stairs from sidewalk later
EDGE_KIND = {}
for _a, _b, _kind in EDGE_SPECS:
    EDGE_KIND[(_a, _b)] = _kind
    EDGE_KIND[(_b, _a)] = _kind

# pristine copy, reset_edges() puts everything back to this
_BASE_EDGES = copy.deepcopy(EDGES)


def all_nodes():
    """Every node id in the graph."""
    return sorted(EDGES.keys())


def get_neighbors(node):
    """Nodes you can walk to directly from here."""
    return list(EDGES[node].keys())


def edge_cost(a, b):
    """Seconds to walk a -> b. inf if blocked or no edge exists."""
    if AVOID_STAIRS and EDGE_KIND.get((a, b)) == "stairs":
        return float("inf")
    return EDGES[a].get(b, float("inf"))


def heuristic(a, b):
    """Estimated seconds a -> b, straight line.
    Admissible because no real path is shorter than a straight line,
    which is what makes A* optimal."""

    x1, y1 = COORDS[a]
    x2, y2 = COORDS[b]
    return math.hypot(x2 - x1, y2 - y1) / WALK_SPEED


def update_edge_cost(a, b, new_cost):
    """Change an edge both ways, RETURNS THE OLD COST.

    Used for crowding, a path getting slower without fully closing."""
    old = EDGES[a][b]
    EDGES[a][b] = new_cost
    EDGES[b][a] = new_cost
    return old


def block_edge(a, b):
    """Obstacle appears. Sets edge to inf both ways, RETURNS THE OLD COST.

    D* Lite needs the old cost to work out which nodes went inconsistent,
    so don't make this return None."""
    old = EDGES[a][b]
    EDGES[a][b] = float("inf")
    EDGES[b][a] = float("inf")
    return old


def reset_edges():
    """Put every edge back to its starting cost. Call between experiments."""
    global EDGES
    EDGES = copy.deepcopy(_BASE_EDGES)


def blocked_edges():
    """Edges that are currently inf, as (a, b) pairs listed once each."""
    out = []
    for a in EDGES:
        for b in EDGES[a]:
            if a < b and EDGES[a][b] == float("inf"):
                out.append((a, b))
    return out


def path_cost(path):
    """Total seconds for a path. inf if any step is blocked."""
    return sum(edge_cost(path[i], path[i + 1]) for i in range(len(path) - 1))


def plot_graph(path=None, blocked=None, title="Northeastern campus", save_to=None):
    """Matplotlib. Nodes at their COORDS, lines for edges, path highlighted,
    blocked edges in red. Just needs to be readable for the report."""
    import matplotlib.pyplot as plt

    if blocked is None:
        blocked = blocked_edges()
    blocked_set = {frozenset(e) for e in blocked}

    fig, ax = plt.subplots(figsize=(13, 11))

    # every walkway, drawn once. stairs get a dashed line.
    for a, b, kind in EDGE_SPECS:
        x1, y1 = COORDS[a]
        x2, y2 = COORDS[b]
        if frozenset((a, b)) in blocked_set:
            ax.plot([x1, x2], [y1, y2], color="red", linewidth=2.5, zorder=2)
        else:
            style = "--" if kind == "stairs" else "-"
            ax.plot([x1, x2], [y1, y2], style, color="0.75", linewidth=1.2, zorder=1)

    # the route on top of everything
    if path:
        px = [COORDS[n][0] for n in path]
        py = [COORDS[n][1] for n in path]
        ax.plot(px, py, color="tab:blue", linewidth=3.5, zorder=3, label="route")
        ax.scatter(px[0], py[0], s=180, color="tab:green", zorder=5, label="start")
        ax.scatter(px[-1], py[-1], s=180, color="tab:red", marker="*", zorder=5, label="goal")

    # junction nodes are smaller since they aren't real places
    for name, (x, y) in COORDS.items():
        junction = name.startswith("jct_")
        ax.scatter(x, y, s=14 if junction else 40, color="0.35", zorder=4)
        if not junction:
            ax.annotate(name.replace("_", " "), (x, y), fontsize=6,
                        xytext=(3, 3), textcoords="offset points")

    ax.set_title(title)
    ax.set_xlabel("meters east")
    ax.set_ylabel("meters north")
    ax.set_aspect("equal")
    if path:
        ax.legend(loc="lower right")
    fig.tight_layout()

    if save_to:
        fig.savefig(save_to, dpi=150)
        plt.close(fig)
    else:
        plt.show()


# ============ ALGORITHMS (B) ============

def astar(start, goal):
    """Plain A*, should be the baseline.
    Returns (path, nodes_expanded). path is a list of node ids including
    start and goal, or [] if no route. nodes_expanded is how many nodes we
    popped off the queue - that's the number we plot."""
    open_list = [(heuristic(start, goal), 0.0, start)]
    came_from = {}
    best_g = {start: 0.0}
    closed = set()
    expanded = 0

    while open_list:
        _, g, node = heapq.heappop(open_list)
        # stale queue entry, we already did this one cheaper
        if node in closed:
            continue
        closed.add(node)
        expanded += 1

        if node == goal:
            path = [node]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path, expanded

        for nb in get_neighbors(node):
            cost = edge_cost(node, nb)
            if cost == float("inf"):
                continue
            new_g = g + cost
            if new_g < best_g.get(nb, float("inf")):
                best_g[nb] = new_g
                came_from[nb] = node
                heapq.heappush(open_list, (new_g + heuristic(nb, goal), new_g, nb))

    return [], expanded


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
