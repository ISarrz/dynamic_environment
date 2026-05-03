#!/usr/bin/env python3
"""
Compare local planners MPPI and RMPPI: quality metrics comparison.

Column layout in data rows (pipe-separated):
  test_id | mppi_status | mppi_ms | mppi_steps | mppi_tort | mppi_turn | mppi_nm
          | rmppi_status | rmppi_ms | rmppi_steps | rmppi_tort | rmppi_turn | rmppi_nm

Input files in results/local/:
  local_static.txt           – output of: test local-static
  local_dynamic_N.txt        – output of: test local-dynamic  (N = objects_per_step)
"""

import re
from pathlib import Path
from typing import Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")

MPPI_COLOR  = "#4878CF"
RMPPI_COLOR = "#D65F5F"


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

class PlannerStats:
    """Aggregate statistics parsed from a single test file."""
    def __init__(self):
        self.times:        List[float] = []
        self.steps:        List[float] = []
        self.tort:         List[float] = []
        self.turn:         List[float] = []
        self.nm:           List[float] = []
        self.found:  int = 0
        self.total:  int = 0

    def success_rate(self) -> float:
        return self.found / self.total if self.total > 0 else 0.0

    def avg(self, lst: List[float]) -> float:
        return sum(lst) / len(lst) if lst else 0.0

    def std(self, lst: List[float]) -> float:
        if len(lst) < 2:
            return 0.0
        m = self.avg(lst)
        return (sum((x - m) ** 2 for x in lst) / len(lst)) ** 0.5


def parse_file(filepath: str):
    """Return (mppi_stats, rmppi_stats) parsed from a test output file."""
    mppi  = PlannerStats()
    rmppi = PlannerStats()
    try:
        with open(filepath) as f:
            for line in f:
                parts = [p.strip() for p in line.split("|")]
                if len(parts) < 13:
                    continue
                try:
                    float(parts[2])  # guard: only data rows have a float in col[2]
                except ValueError:
                    continue

                # MPPI columns: 1=status, 2=ms, 3=steps, 4=tort, 5=turn, 6=nm
                mppi.total  += 1
                mppi.times.append(float(parts[2]))
                if parts[1].strip() == "Path found":
                    mppi.found += 1
                    mppi.steps.append(float(parts[3]))
                    mppi.tort.append(float(parts[4]))
                    mppi.turn.append(float(parts[5]))
                    mppi.nm.append(float(parts[6]))

                # RMPPI columns: 7=status, 8=ms, 9=steps, 10=tort, 11=turn, 12=nm
                rmppi.total += 1
                rmppi.times.append(float(parts[8]))
                if parts[7].strip() == "Path found":
                    rmppi.found += 1
                    rmppi.steps.append(float(parts[9]))
                    rmppi.tort.append(float(parts[10]))
                    rmppi.turn.append(float(parts[11]))
                    rmppi.nm.append(float(parts[12]))
    except FileNotFoundError:
        pass
    return mppi, rmppi


# ---------------------------------------------------------------------------
# Static comparison (2 × 3 subplots)
# ---------------------------------------------------------------------------

def plot_static(results_dir: Path, output_path: Path) -> None:
    mppi, rmppi = parse_file(str(results_dir / "local_static.txt"))
    if not mppi.total:
        print("No local static data found, skipping static plot")
        return

    labels = ["MPPI", "RMPPI"]
    colors = [MPPI_COLOR, RMPPI_COLOR]
    x = np.arange(2)
    w = 0.4

    metrics = [
        ("Доля успешных навигаций (%)",     "%",     [mppi.success_rate() * 100, rmppi.success_rate() * 100],  True),
        ("Среднее число шагов",             "шаги",  [mppi.avg(mppi.steps), rmppi.avg(rmppi.steps)],           False),
        ("Стабильность (σ шагов)",          "шаги",  [mppi.std(mppi.steps), rmppi.std(rmppi.steps)],           False),
        ("Извилистость пути",               "",      [mppi.avg(mppi.tort),  rmppi.avg(rmppi.tort)],            False),
        ("Плавность (ср. угол поворота)",   "°",     [mppi.avg(mppi.turn),  rmppi.avg(rmppi.turn)],            False),
        ("Риск (доля шагов у препятствий)", "",      [mppi.avg(mppi.nm),    rmppi.avg(rmppi.nm)],              False),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    fig.suptitle("Статическая среда: сравнение MPPI и RMPPI",
                 fontsize=14, fontweight="bold")

    for ax, (title, ylabel, vals, pct) in zip(axes.flat, metrics):
        bars = ax.bar(x, vals, width=w, color=colors)
        ax.set_title(title, fontsize=11)
        if ylabel:
            ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.grid(axis="y", alpha=0.3)
        if pct:
            ax.set_ylim(0, 110)
        for bar, val in zip(bars, vals):
            fmt = f"{val:.1f}%" if pct else f"{val:.3f}"
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() * 1.02,
                    fmt, ha="center", va="bottom", fontsize=10, fontweight="bold")

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches="tight")
    print(f"Local static comparison saved to: {output_path}")
    plt.close()


# ---------------------------------------------------------------------------
# Dynamic comparison (2 × 3 subplots)
# ---------------------------------------------------------------------------

def _load_dynamic(results_dir: Path) -> Dict[int, tuple]:
    """Returns {n: (mppi_stats, rmppi_stats)} for each local_dynamic_N.txt."""
    data = {}
    for lpath in sorted(results_dir.glob("local_dynamic_*.txt")):
        m = re.search(r"_(\d+)\.txt$", lpath.name)
        if not m:
            continue
        n = int(m.group(1))
        mppi, rmppi = parse_file(str(lpath))
        if mppi.total:
            data[n] = (mppi, rmppi)
    return data


def plot_dynamic(results_dir: Path, output_path: Path) -> None:
    data = _load_dynamic(results_dir)
    if not data:
        print("No local dynamic data found, skipping dynamic plot")
        return

    xs = sorted(data.keys())

    def series(fn):
        return (
            [fn(data[x][0]) for x in xs],
            [fn(data[x][1]) for x in xs],
        )

    panels = [
        ("Доля успешных навигаций (%)",     "%",   series(lambda s: s.success_rate() * 100)),
        ("Среднее число шагов",             "шаги",series(lambda s: s.avg(s.steps))),
        ("Стабильность (σ шагов)",          "шаги",series(lambda s: s.std(s.steps))),
        ("Извилистость пути",               "",    series(lambda s: s.avg(s.tort))),
        ("Плавность (ср. угол поворота)",   "°",   series(lambda s: s.avg(s.turn))),
        ("Риск (доля шагов у препятствий)", "",    series(lambda s: s.avg(s.nm))),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle("Динамическая среда: сравнение MPPI и RMPPI",
                 fontsize=14, fontweight="bold")

    xlabel = "Препятствий изменяется за ход"

    for ax, (title, ylabel, (ym, yr)) in zip(axes.flat, panels):
        ax.plot(xs, ym, marker="o", linestyle="-",  label="MPPI",  color=MPPI_COLOR,  linewidth=2)
        ax.plot(xs, yr, marker="s", linestyle="--", label="RMPPI", color=RMPPI_COLOR, linewidth=2)
        ax.set_title(title, fontsize=11)
        ax.set_xlabel(xlabel)
        if ylabel:
            ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches="tight")
    print(f"Local dynamic comparison saved to: {output_path}")
    plt.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    script_dir  = Path(__file__).parent
    root_dir    = script_dir.parent.parent
    results_dir = root_dir / "results"
    local_dir   = results_dir / "local"

    if not local_dir.exists():
        print(f"No local results directory found at {local_dir}")
        print("Run results/cmd/run_local_tests.sh first.")
        return

    plot_static(local_dir,  results_dir / "local_static_comparison.png")
    plot_dynamic(local_dir, results_dir / "local_dynamic_comparison.png")
    print("Done.")


if __name__ == "__main__":
    main()
