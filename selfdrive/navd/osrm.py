import requests

from openpilot.selfdrive.navd.helpers import Coordinate


OSRM_ROUTE_URL = "https://router.project-osrm.org/route/v1/driving/{coords}"
OSRM_MAX_DURATION_SLOWDOWN = 1.15
OSRM_NON_ACTIONABLE_MANEUVERS = {"arrive", "depart", "new name"}
OSRM_ORIGIN_BEARING_RANGE_DEGREES = 90


def _banner_from_osrm_step(step, distance_along_geometry):
  maneuver = step.get("maneuver", {})
  modifier = maneuver.get("modifier", "")
  maneuver_type = maneuver.get("type", "")
  name = step.get("name") or maneuver_type.replace("_", " ").title()

  primary = {"text": name}
  if maneuver_type:
    primary["type"] = maneuver_type
  if modifier:
    primary["modifier"] = modifier

  return [{
    "distanceAlongGeometry": distance_along_geometry,
    "primary": primary,
  }]


def _normalize_osrm_step(step, next_step):
  geometry = step.get("geometry", {})
  coords = geometry.get("coordinates", [])
  if len(coords) < 2:
    maneuver = step.get("maneuver", {})
    location = maneuver.get("location")
    if location is not None:
      coords = [location, location]

  return {
    "bannerInstructions": _banner_from_osrm_step(next_step, step.get("distance", 0.0)) if next_step is not None else [],
    "distance": step.get("distance", 0.0),
    "duration": step.get("duration", 0.0),
    "duration_typical": step.get("duration", 0.0),
    "geometry": {"coordinates": coords},
    "intersections": step.get("intersections", []),
    "maneuver": step.get("maneuver", {}),
    "name": step.get("name", ""),
  }


def _route_maneuver_count(route):
  count = 0
  for leg in route.get("legs", []):
    for step in leg.get("steps", []):
      maneuver_type = step.get("maneuver", {}).get("type", "")
      if maneuver_type and maneuver_type not in OSRM_NON_ACTIONABLE_MANEUVERS:
        count += 1
  return count


def _choose_route(routes):
  fastest_duration = min(route.get("duration", float("inf")) for route in routes)
  duration_limit = fastest_duration * OSRM_MAX_DURATION_SLOWDOWN
  candidates = [route for route in routes if route.get("duration", float("inf")) <= duration_limit]
  return min(candidates, key=lambda route: (_route_maneuver_count(route), route.get("duration", float("inf")), route.get("distance", float("inf"))))


def request_osrm_route(origin: Coordinate, destination: Coordinate, origin_bearing: float | None = None):
  coords = f"{origin.longitude},{origin.latitude};{destination.longitude},{destination.latitude}"
  params = {
    "alternatives": "true",
    "geometries": "geojson",
    "overview": "full",
    "steps": "true",
  }
  if origin_bearing is not None:
    params["bearings"] = f"{(origin_bearing + 360) % 360:.0f},{OSRM_ORIGIN_BEARING_RANGE_DEGREES};"

  resp = requests.get(OSRM_ROUTE_URL.format(coords=coords), params=params, timeout=10)
  resp.raise_for_status()
  data = resp.json()

  routes = data.get("routes") or []
  if not routes:
    return None, data

  route = _choose_route(routes)
  legs = route.get("legs") or []
  if not legs:
    return None, data

  steps = []
  osrm_steps = legs[0].get("steps", [])
  for idx, step in enumerate(osrm_steps):
    next_step = osrm_steps[idx + 1] if idx + 1 < len(osrm_steps) else None
    normalized = _normalize_osrm_step(step, next_step)
    if normalized["geometry"]["coordinates"]:
      steps.append(normalized)

  route["legs"][0]["steps"] = steps
  route["legs"][0]["annotation"] = {"maxspeed": []}
  return route, data
