# CS4100 Campus Navigation Project

A navigation system for Northeastern's Boston campus. You give it a start and a
destination and it returns the fastest walking route, then updates that route if
something changes while you are already walking, like a walkway closing for
construction or a path getting jammed between classes.

## Why we are doing this

Google Maps can get you from building to building, but it treats campus like any
other road network. It does not know about the shortcut through Snell, it does
not know Forsyth is closed for construction this week, and it does not know that
the bridge over the train tracks to ISEC is stairs only. Students end up figuring
it out themselves or asking someone.

The other half of this is accessibility. If you cannot use stairs, the shortest
route is often not a route you can actually take. Most tools treat that as a
filter you turn on at the end. We treat it as part of the cost model, so the
router just never hands you a path you cannot use.

## The idea

Model campus as a graph. Buildings and walkway corners are nodes, sidewalks
between them are edges, and each edge costs however many seconds it takes to walk
it. Finding a route is then a shortest path problem.

We use two algorithms:

- **A\*** finds the best route from scratch. This is our baseline.
- **D\* Lite** repairs an existing route when the map changes, instead of
  starting over. This is the interesting one, because a student walking to class
  who hits a closed path does not need the whole search redone, only the part
  that actually changed.

The experiment is measuring whether that repair is actually cheaper than just
rerunning A\*, and by how much.

## Files

- `campus-routing/campus_nav.py` is the campus graph, the A\* search, and the
  D\* Lite class
- `campus-routing/check_graph.py` checks the graph for mistakes, run it after
  editing coordinates or edges
- `campus-routing/experiments.py` walks an agent along a route, breaks the map
  partway through, and records what each planner had to do to recover
- `campus-routing/results/` holds the csv logs and the plots

## Running it

```
cd campus-routing
python3 check_graph.py     # should print all checks passed
python3 experiments.py     # writes results/replans.csv and results/summary.csv
```

`experiments.py` runs the A\* baseline on its own, so it works before D\* Lite is
finished. Once `DStarLite` is implemented it gets detected automatically and both
planners run side by side with no changes needed.

## How the graph works

Nodes are stored as lat/lon in `LATLON` and converted to meters. Coordinates are
approximate and should be checked against the real campus map before we put any
figures in the report.

Edge costs are in seconds, not meters. We do not type them in by hand. Each edge
says what kind of path it is, and the cost comes out of the straight line
distance times a slowdown factor for that kind:

- `walk` is an open sidewalk, factor 1.0
- `busy` is a hallway or door that jams up between classes, factor 1.25
- `ramp` is step free but takes the long way round, factor 1.15
- `stairs` is slower and unusable in accessible mode, factor 1.35

Every factor has to stay at or above 1.0. If a path were ever faster than the
straight line between its endpoints, the A\* heuristic would start overestimating
and A\* would quietly return routes that are not actually the fastest. It would
not crash, it would just be wrong, which is worse. `check_graph.py` tests for
this.

## Accessible routing

Set `AVOID_STAIRS = True` and `edge_cost` starts returning infinity for stairs
edges, so both algorithms route around them without needing any special case.
`check_graph.py` confirms campus is still fully connected with stairs turned off,
so nobody gets stranded.

Example: from the pedestrian crossing to ISEC, the normal route is the bridge
stairs at 0.8 minutes. In accessible mode it goes around through Columbus garage
instead, which takes 2.3 minutes.
