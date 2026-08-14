# CS4100 Campus Navigation Project

A navigation system for Northeastern's Boston campus. You give it a start and a
destination and it returns the fastest walking route, then updates that route if
something changes while you are already walking, like a walkway closing for
construction or a path getting jammed between classes.

## Why we are doing this

Google Maps can get you from building to building, but it treats campus like any
other road network. It does not know a walkway is closed for construction this
week, it does not know which paths jam up between classes, and it has no idea
which routes involve stairs. Students end up figuring it out themselves or
asking someone.

The other half of this is accessibility. If you cannot use stairs, the fastest
route on paper is not always the route you would actually want. Most tools treat
that as a filter you turn on at the end. We treat it as part of the cost model,
so the router leans toward step free paths on its own.

## How it works

Campus is modeled as a graph. Buildings and walkway corners are nodes (60 of
them), sidewalks between them are edges (82), and each edge costs however many
seconds it takes to walk it. Finding a route is then a shortest path problem.

The pipeline goes:

1. `campus_nav.py` holds the graph. Node positions are real lat/lon pulled off
   the map, converted into meters. Edge costs are worked out from straight line
   distance and walking speed, we never type seconds in by hand.
2. A* searches the graph for the fastest route. It uses straight line time as
   its heuristic, which never overestimates, so the routes it returns are
   actually the fastest ones.
3. While the agent walks, the map can change. An edge can be blocked
   (construction) or get more expensive (crowds). The planner is told what
   changed and replans.
4. The naive way to replan is rerunning A* from scratch every time. D* Lite
   instead keeps its search state and only repairs the part that changed. Our
   experiments measure how much work each one does for the same situations.

## Files

everything lives in `campus-routing/`

- `campus_nav.py` is the campus graph and the plain A* search, plus plotting
- `algorithm.py` is Kriti's algorithm file, her A* and the D* Lite class
- `check_graph.py` checks the graph for mistakes, run it after editing
  coordinates or edges
- `experiments.py` walks an agent along a route, breaks the map partway
  through, and records nodes expanded and time per replan
- `compare_plots.py` turns the experiment csv into the bar charts for the report
- `trip.py` is routes with stops along the way, like grabbing coffee before
  class, and works out what time to leave
- `make_demo.py` records the three demo videos, a closure, a crowd, and a trip with stops
- `results/` holds the csv logs, plots, and the demo videos

## Getting it running

You need python3 with matplotlib (`pip3 install matplotlib`). For the demo
video you also need ffmpeg (`brew install ffmpeg` on mac), without it you still
get a gif.

```
cd campus-routing
python3 check_graph.py       # sanity checks the graph, run this first
python3 experiments.py       # runs all 6 scenarios, writes results/*.csv
python3 compare_plots.py     # makes the comparison charts from the csv
python3 trip.py              # multi stop trip demo with leave-by times
python3 make_demo.py         # records the three demo videos
```

`experiments.py` runs both planners side by side, the naive A* baseline and
D* Lite from `algorithm.py`, over the same six scenarios.

## Making the videos and plots yourself

Everything in `results/` is generated, so you can delete the folder and build it
all back.

The three demo videos come from `make_demo.py`:

```
cd campus-routing
python3 make_demo.py
```

That writes `demo_closure.mp4`, `demo_crowd.mp4` and `demo_stops.mp4` into
`results/` and prints the frame count for each one.

The mp4 part needs ffmpeg. Check with `ffmpeg -version`, and if you don't have
it:

```
brew install ffmpeg       # mac
sudo apt install ffmpeg   # linux
```

Without ffmpeg it falls back to a gif. The gifs are bigger and look worse, so
install ffmpeg if you can.

Three settings at the top of `make_demo.py` change how the videos look:

- `FPS` is playback speed, 3 is slow enough to read the side panel
- `HOLD` is how many frames it freezes on the moment something happens
- `TWEEN` is the in between frames while walking one edge, higher is smoother
  but makes a longer video

The bar charts come from `compare_plots.py`, but run `experiments.py` first
because it reads `results/summary.csv`.

`campus_graph.png` is the plain map of the whole graph. There's no script for
it, it's a one liner:

```
python3 -c "import campus_nav; campus_nav.plot_graph(save_to='results/campus_graph.png')"
```

`plot_graph` also takes a `path` to draw a route on top and a `title`.

## How the graph data works

Nodes are stored as lat/lon in `LATLON` in `campus_nav.py`. To add a building,
right click it on google maps, copy the coordinates, add it to `LATLON` and add
its walkways to `EDGE_SPECS`. Then run `check_graph.py`, it catches one way
edges, unknown names, stranded buildings, that kind of thing.

Edge costs come from straight line distance times a slowdown factor per path
kind:

- `walk` is an open sidewalk, factor 1.0
- `stairs` has steps on it, factor 1.35 since climbing is slower

Every factor has to stay at or above 1.0. If a path were ever faster than the
straight line between its endpoints, the A* heuristic would start
overestimating and A* could return routes that are not actually the fastest.
`check_graph.py` tests for this.

One known limitation: real campus paths curve around buildings but our costs
use the straight line, so walking times come out a little optimistic. Where a
path bends enough to matter we added corner nodes (the `jct_` ones), like the
West Village to International Village walk which goes down Leon St and along
Ruggles St instead of straight across.

## Accessible routing

Every stairs spot on campus has a ramp or elevator next to it, so step free
routing does not ban stairs edges. Instead, setting `AVOID_STAIRS = True` makes
stairs edges cost extra (`STEP_FREE_PENALTY`), which is the time it takes to go
around on the ramp. Routes then lean toward step free paths on their own, and
if a ramp or elevator is closed for construction, `block_edge` handles it like
any other closure and the route goes around it.

`check_graph.py` confirms step free mode keeps the whole campus reachable.

## The experiments

Six scenarios, each one walks the agent from Speare Hall across campus to West
Village A (except the crowding one which goes to Ell Hall). Partway through we
block an edge or spike its cost, the planner replans, and we log how many nodes
it expanded and how long the replan took.

- control with no changes, to have something to compare against
- blocked path near the start, near the goal, and mid route
- three changes in one trip
- a crowd spike that makes a path slow but not blocked

When the map changes the naive planner redoes the whole search, while D* Lite
only repairs what changed. Nodes expanded is the number that shows the
difference.

## Results

Total nodes expanded across the whole trip, from `results/summary.csv`:

| scenario | naive A* | D* Lite |
| --- | --- | --- |
| control_no_change | 42 | 44 |
| block_near_start | 82 | 53 |
| block_mid_route | 74 | 48 |
| block_near_goal | 47 | 47 |
| multiple_changes | 127 | 64 |
| crowd_spike | 38 | 39 |

D* Lite wins when there is something to repair, and the more changes there are
the bigger the gap, since the naive planner pays for a full search every time
and D* Lite doesn't. `multiple_changes` has three events in one trip and is
where they separate most, 127 against 64.

The control has nothing to repair, so it comes out even, D* Lite is two nodes
worse because the first search has to run either way. `crowd_spike` is
close for the same reason, a slower edge that doesn't block anything barely
changes the route. `block_near_goal` ties because the agent is nearly there and
neither planner has much left to search.

Both planners produce the same `travel_minutes` in every scenario, so D* Lite
is not saving work by returning worse routes.

The milliseconds in `summary.csv` do not favor D* Lite, it is slower in wall
clock in all six. On a 60 node graph a full A* is already under a millisecond,
so what gets measured is mostly python overhead per node, and D* Lite does more
work per node. Nodes expanded is the number that reflects the algorithms.

A 60 node graph is also small enough that the savings stay small in absolute
terms. The gap should widen on a bigger graph, but we have not tested that.
