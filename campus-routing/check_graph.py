"""Sanity checks on the campus graph. Run this after editing COORDS or
EDGE_SPECS, before trusting any experiment results."""

import math

import campus_nav as cn


def check_symmetric():
    """Every edge has to exist both ways with the same cost."""
    bad = []
    for a in cn.EDGES:
        for b, cost in cn.EDGES[a].items():
            if cn.EDGES.get(b, {}).get(a) != cost:
                bad.append((a, b))
    return bad


def check_no_duplicates():
    """Same pair listed twice in EDGE_SPECS, probably a copy paste slip."""
    seen = set()
    dupes = []
    for a, b, _ in cn.EDGE_SPECS:
        key = frozenset((a, b))
        if key in seen:
            dupes.append((a, b))
        seen.add(key)
    return dupes


def check_unknown_nodes():
    """Edges pointing at a node that isn't in COORDS."""
    return [(a, b) for a, b, _ in cn.EDGE_SPECS
            if a not in cn.COORDS or b not in cn.COORDS]


def reachable_from(start):
    """Plain flood fill, ignores costs."""
    seen = {start}
    stack = [start]
    while stack:
        node = stack.pop()
        for nb in cn.get_neighbors(node):
            if nb not in seen and cn.edge_cost(node, nb) != float("inf"):
                seen.add(nb)
                stack.append(nb)
    return seen


def check_connected():
    """Nodes you can't walk to from snell library."""
    return sorted(set(cn.all_nodes()) - reachable_from("snell_library"))


def check_admissible():
    """The heuristic must never overestimate a single edge, otherwise A*
    can return a route that isn't actually the fastest."""
    bad = []
    for a, b, _ in cn.EDGE_SPECS:
        if cn.heuristic(a, b) > cn.EDGES[a][b] + 1e-9:
            bad.append((a, b))
    return bad


# edges we checked on the map and are really that long
KNOWN_LONG = {
    # the walk out to mass ave and along columbus to squashbusters
    frozenset(("jct_columbus_ave", "squashbusters")),
    # the sheraton really is that far up mass ave
    frozenset(("sheraton", "jct_mass_ave")),
}


def check_edge_lengths(limit=350):
    """Really long edges usually mean a typo in the coordinates."""
    out = []
    for a, b, _ in cn.EDGE_SPECS:
        if frozenset((a, b)) in KNOWN_LONG:
            continue
        x1, y1 = cn.COORDS[a]
        x2, y2 = cn.COORDS[b]
        d = math.hypot(x2 - x1, y2 - y1)
        if d > limit:
            out.append((a, b, round(d)))
    return out


def check_stairs_have_alternatives():
    """Step free mode has to keep campus connected, and the penalty can't
    be under 1.0 or the heuristic starts overestimating."""
    if cn.STEP_FREE_PENALTY < 1.0:
        return ["STEP_FREE_PENALTY is under 1.0"]
    cn.AVOID_STAIRS = True
    stranded = sorted(set(cn.all_nodes()) - reachable_from("snell_library"))
    cn.AVOID_STAIRS = False
    return stranded


def main():
    checks = [
        ("edges symmetric", check_symmetric()),
        ("no duplicate edges", check_no_duplicates()),
        ("all nodes known", check_unknown_nodes()),
        ("graph connected", check_connected()),
        ("heuristic admissible", check_admissible()),
        ("no absurd edge lengths", check_edge_lengths()),
        ("step free route exists", check_stairs_have_alternatives()),
    ]

    print(f"{len(cn.all_nodes())} nodes, {len(cn.EDGE_SPECS)} edges\n")
    failed = 0
    for name, problems in checks:
        if problems:
            failed += 1
            print(f"FAIL  {name}: {problems}")
        else:
            print(f"ok    {name}")

    print()
    print("all checks passed" if failed == 0 else f"{failed} check(s) failed")
    return failed


if __name__ == "__main__":
    raise SystemExit(main())
