"""Regenerates elastic-layout.json from the dashboard keys used in the code. Run from the repo root: python tools/make_elastic_layout.py"""
import json, re
G = 128
GREEN, RED, GRAY, ORANGE, BLUE, YELLOW = 4283215696, 4294198070, 4288585374, 4294940672, 4280391411, 4294961979
SD = "/SmartDashboard/"

def w(title, x, y, wd, ht, typ, topic, **props):
    p = {"topic": topic, "period": 0.06}
    p.update(props)
    return {"title": title, "x": x * G, "y": y * G, "width": wd * G, "height": ht * G, "type": typ, "properties": p}

def box(title, x, y, key, true_color=GREEN, false_color=GRAY, wd=1, ht=1):
    return w(title, x, y, wd, ht, "Boolean Box", SD + key, data_type="boolean",
             true_color=true_color, false_color=false_color, true_icon="None", false_icon="None")

def text(title, x, y, key, wd=2, ht=1, data_type="string", editable=False):
    return w(title, x, y, wd, ht, "Text Display", SD + key, data_type=data_type, show_submit_button=editable)

def toggle(title, x, y, key, wd=2):
    return w(title, x, y, wd, 1, "Toggle Switch", SD + key, data_type="boolean")

def button(title, x, y, key, wd=2):
    return w(title, x, y, wd, 1, "Toggle Button", SD + key, data_type="boolean")

field = w("Field", 4, 0, 5, 3, "Field", SD + "Field", field_game="Rebuilt", robot_width=0.85, robot_length=0.85,
          show_other_objects=True, show_trajectories=True, field_rotation=0.0, robot_color=RED,
          trajectory_color=4294967295, show_robot_outside_widget=True)

driver = [
    text("SHOT MODE", 0, 0, "Driver/Shot Mode", wd=2),
    box("SCORING", 2, 0, "Driver/Scoring"),
    box("PASSING", 3, 0, "Driver/Passing", true_color=BLUE),
    box("BALL IN SHOOTER", 0, 1, "Driver/Ball In Shooter", wd=2),
    box("TRENCH", 2, 1, "Driver/Trench", true_color=ORANGE),
    box("READY", 3, 1, "Driver/Ready"),
    text("HUB", 0, 2, "Driver/Hub", wd=2),
    box("HUB ON", 2, 2, "Driver/Hub Active", false_color=RED),
    box("HOLDING FIRE", 3, 2, "Driver/Holding Fire", true_color=YELLOW),
    field,
    w("Auto Chooser", 9, 0, 2, 1, "ComboBox Chooser", SD + "Auto Chooser", sort_options=False),
    text("Match Time", 9, 1, "Driver/Match Time", wd=1, data_type="double"),
    text("Balls Shot", 10, 1, "Driver/Balls Shot", wd=1, data_type="double"),
    text("Flywheel Offset (rps)", 9, 2, "Driver/Flywheel Offset", wd=1, data_type="double"),
    text("Turret Offset (deg)", 10, 2, "Driver/Turret Offset", wd=1, data_type="double"),
    box("HEALTHY", 0, 3, "Driver/Healthy", false_color=RED),
    text("Robot Health", 1, 3, "Driver/Robot Health", wd=10),
    text("Front Camera", 0, 4, "Vision/Front Status", wd=4),
    text("Side Camera", 4, 4, "Vision/Side Status", wd=5),
    text("Hood Homing", 9, 4, "Driver/Hood Homing", wd=2),
]
shots = [
    button("MARK MADE", 0, 0, "Driver/Mark Made"),
    button("MARK MISSED", 2, 0, "Driver/Mark Missed"),
    text("Unmarked", 0, 1, "Driver/Shots Unmarked", wd=1, data_type="double"),
    text("Made", 1, 1, "Driver/Shots Made", wd=1, data_type="double"),
    text("Missed", 2, 1, "Driver/Shots Missed", wd=1, data_type="double"),
    text("Balls Shot", 3, 1, "Driver/Balls Shot", wd=1, data_type="double"),
    w("Flywheel Stator Amps", 4, 0, 5, 3, "Graph", SD + "Driver/Flywheel Amps", data_type="double",
      time_displayed=10.0, color=ORANGE, line_width=2.0),
    box("BALL IN SHOOTER", 0, 2, "Driver/Ball In Shooter", wd=2),
    box("AT SPEED", 2, 2, "Driver/Flywheel At Speed"),
    box("FEEDING", 3, 2, "Driver/Feeding"),
    toggle("Wait For Ready", 0, 3, "Driver/Wait For Ready"),
    toggle("Hold Fire When Hub Off", 2, 3, "Driver/Hold Fire When Hub Off"),
    toggle("Tilt Compensation", 4, 3, "Driver/Tilt Compensation"),
    text("Tilt Turret Fix (deg)", 6, 3, "Driver/Tilt Turret Fix Deg", wd=2, data_type="double"),
]

# Tuning tab: every Tuning value, editable. Keys come straight from Tuning.java so they can't drift.
src = open("src/main/java/frc/robot/Tuning.java").read()
keys = re.findall(r'add\("([^"]+)"', src)
table = [0, 1, 1.5, 2, 3, 4, 5, 6, 8]
consts = open("src/main/java/frc/robot/Constants.java").read()
block = consts[consts.index("SHOT_TABLE = {"):]
block = block[:block.index("};")]
dists = re.findall(r"\{(\d+(?:\.\d+)?),", block)
expanded = []
for d in dists:
    expanded.append(f"Shot Table/{float(d)} m rps")
for d in dists:
    expanded.append(f"Shot Table/{float(d)} m tof s")
expanded += [k for k in keys if not k.startswith("Shot Table/")]
tuning = [button("PRINT VALUES", 0, 0, "Tuning/Print Values", wd=3)]
for i, k in enumerate(expanded):
    col, row = divmod(i, 5)
    tuning.append(text(k, col * 3, row + 1, "Tuning/" + k, wd=3, data_type="double", editable=True))
tuning.append(w("SysId Mechanism", 9, 0, 2, 1, "ComboBox Chooser", SD + "SysId Mechanism", sort_options=False))

layout = {"version": 1.0, "grid_size": G, "tabs": [
    {"name": "Driver", "grid_layout": {"layouts": [], "containers": driver}},
    {"name": "Shots", "grid_layout": {"layouts": [], "containers": shots}},
    {"name": "Tuning", "grid_layout": {"layouts": [], "containers": tuning}},
]}
open("elastic-layout.json", "w").write(json.dumps(layout, indent=2) + "\n")
print({t["name"]: len(t["grid_layout"]["containers"]) for t in layout["tabs"]})
print(expanded[:3], len(expanded))
