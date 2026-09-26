"""Suggest a new shot table from the robot's shot log.

Get the log off the robot:   scp lvuser@10.77.63.2:shots.csv .
Then run:                    python tools/analyze_shots.py shots.csv

For every distance in Constants.Shooting.SHOT_TABLE it looks at MADE shots within +/-0.5 m and
suggests the median flywheel speed actually used (goal_rps already includes any D-pad offset the
driver dialed in). It also prints the make rate per distance so you can see where the table is weak.
"""
import csv
import re
import statistics
import sys
from pathlib import Path

WINDOW_M = 0.5


def shot_table():
    src = Path(__file__).resolve().parent.parent / "src/main/java/frc/robot/Constants.java"
    text = src.read_text()
    block = text[text.index("SHOT_TABLE = {"):]
    block = block[:block.index("};")]
    return [tuple(float(v) for v in row) for row in re.findall(r"\{([\d.]+), ([\d.]+), ([\d.]+)\}", block)]


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("no shots in", path)
        return
    table = shot_table()
    print(f"{len(rows)} shots: {sum(r['result'] == 'made' for r in rows)} made, "
          f"{sum(r['result'] == 'missed' for r in rows)} missed, {sum(r['result'] == 'unknown' for r in rows)} unknown\n")
    print(f"{'dist':>5} {'table rps':>9} {'made':>5} {'missed':>6} {'rate':>5} {'suggested rps':>13}")
    suggestions = []
    for dist, rps, tof in table:
        near = [r for r in rows if abs(float(r["distance_m"]) - dist) <= WINDOW_M]
        made = [float(r["goal_rps"]) for r in near if r["result"] == "made"]
        missed = sum(r["result"] == "missed" for r in near)
        total = len(made) + missed
        rate = f"{100 * len(made) / total:.0f}%" if total else "-"
        new = statistics.median(made) if len(made) >= 3 else rps
        flag = "" if len(made) >= 3 else " (need 3+ makes)"
        print(f"{dist:5.1f} {rps:9.1f} {len(made):5d} {missed:6d} {rate:>5} {new:13.1f}{flag}")
        suggestions.append((dist, new, tof))
    print("\nPaste into Constants.Shooting.SHOT_TABLE:")
    for dist, rps, tof in suggestions:
        print(f"            {{{dist}, {rps:.1f}, {tof:.2f}}},")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "shots.csv")
