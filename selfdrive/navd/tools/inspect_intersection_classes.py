#!/usr/bin/env python3
"""Compare intersection['classes'] vs intersection['mapbox_streets_v8'] on a real route."""
import json, os, sys, urllib.request, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
TOKEN_PATH = os.path.join(HERE, "..", "mapbox_token")

with open(TOKEN_PATH) as f:
  token = f.read().strip()

# Saudi Arabia: Riyadh point A -> point B (lon,lat)
coords = "46.593946,24.738824;46.703538,24.593049"
url = f"https://api.mapbox.com/directions/v5/mapbox/driving-traffic/{coords}"
params = {
  "access_token": token,
  "annotations": "maxspeed",
  "geometries": "geojson",
  "overview": "full",
  "steps": "true",
  "banner_instructions": "true",
}
full_url = url + "?" + urllib.parse.urlencode(params)
with urllib.request.urlopen(full_url) as resp:
  data = json.load(resp)

with open("/tmp/mapbox_response.json", "w") as f:
  json.dump(data, f, indent=2)
print("Full response saved to /tmp/mapbox_response.json")

steps = data["routes"][0]["legs"][0]["steps"]
print(f"\nSteps: {len(steps)}")

classes_seen = set()
streets_v8_class_seen = set()
classes_count = 0
streets_v8_count = 0
mismatches = []
sample_with_classes = None
sample_with_streets_v8 = None

for step_idx, step in enumerate(steps):
  for int_idx, inter in enumerate(step.get("intersections", [])):
    cls_arr = inter.get("classes")
    streets_v8 = inter.get("mapbox_streets_v8")
    if cls_arr:
      classes_count += 1
      for c in cls_arr:
        classes_seen.add(c)
      if sample_with_classes is None:
        sample_with_classes = (step_idx, int_idx, inter)
    if streets_v8:
      streets_v8_count += 1
      v8_cls = streets_v8.get("class") if isinstance(streets_v8, dict) else None
      if v8_cls:
        streets_v8_class_seen.add(v8_cls)
      if sample_with_streets_v8 is None:
        sample_with_streets_v8 = (step_idx, int_idx, inter)
    if cls_arr and streets_v8:
      v8_cls = streets_v8.get("class") if isinstance(streets_v8, dict) else None
      if v8_cls and v8_cls not in cls_arr:
        mismatches.append((step_idx, int_idx, cls_arr, v8_cls))

print(f"\nIntersections with 'classes':            {classes_count}")
print(f"Intersections with 'mapbox_streets_v8':  {streets_v8_count}")
print(f"\nDistinct values in 'classes':            {sorted(classes_seen)}")
print(f"Distinct values in 'mapbox_streets_v8.class': {sorted(streets_v8_class_seen)}")
print(f"\nMismatches (v8.class not in classes):    {len(mismatches)}")
for m in mismatches[:5]:
  print(f"  step={m[0]} int={m[1]} classes={m[2]} v8.class={m[3]}")

if sample_with_classes:
  s, i, inter = sample_with_classes
  print(f"\n--- Sample intersection with 'classes' (step {s} int {i}) ---")
  print(json.dumps(inter, indent=2))

if sample_with_streets_v8 and sample_with_streets_v8 is not sample_with_classes:
  s, i, inter = sample_with_streets_v8
  print(f"\n--- Sample intersection with 'mapbox_streets_v8' (step {s} int {i}) ---")
  print(json.dumps(inter, indent=2))

# Specifically hunt for off-ramps
print("\n--- Looking for off-ramp signals ---")
for step_idx, step in enumerate(steps):
  for int_idx, inter in enumerate(step.get("intersections", [])):
    cls_arr = inter.get("classes", [])
    streets_v8 = inter.get("mapbox_streets_v8") or {}
    v8_cls = streets_v8.get("class") if isinstance(streets_v8, dict) else None
    if "motorway_link" in cls_arr or v8_cls == "motorway_link":
      print(f"  step={step_idx} int={int_idx} classes={cls_arr} v8.class={v8_cls}")
