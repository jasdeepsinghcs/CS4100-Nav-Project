# Makes the comparison plots for the report. Run experiments.py first.
# Plots whichever planners are in the csv, so it works before D* Lite is done.

import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = list(csv.DictReader(open("results/summary.csv")))
planners = sorted({r["planner"] for r in rows})
scenarios = []
for r in rows:
    if r["scenario"] not in scenarios:
        scenarios.append(r["scenario"])


def numbers(planner, field):
    vals = {r["scenario"]: float(r[field]) for r in rows if r["planner"] == planner}
    return [vals.get(s, 0) for s in scenarios]


plots = [
    ("total_expanded", "nodes expanded", "plot_expanded.png"),
    ("total_millis", "replanning time (ms)", "plot_time.png"),
]

for field, label, fname in plots:
    fig, ax = plt.subplots(figsize=(9, 5))
    width = 0.8 / len(planners)
    for i, p in enumerate(planners):
        xs = [j + i * width for j in range(len(scenarios))]
        ax.bar(xs, numbers(p, field), width, label=p)
    ax.set_xticks([j + width * (len(planners) - 1) / 2 for j in range(len(scenarios))])
    ax.set_xticklabels([s.replace("_", "\n") for s in scenarios], fontsize=8)
    ax.set_ylabel(label)
    ax.set_title(f"Total {label} per scenario")
    ax.legend()
    fig.tight_layout()
    fig.savefig("results/" + fname, dpi=150)
    print("wrote results/" + fname)
