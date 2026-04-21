import requests

from openpilot.selfdrive.navd.helpers import Coordinate


OSRM_ROUTE_URL = "https://router.project-osrm.org/route/v1/driving/{coords}"


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


def request_osrm_route(origin: Coordinate, destination: Coordinate):
  coords = f"{origin.longitude},{origin.latitude};{destination.longitude},{destination.latitude}"
  params = {
    "alternatives": "false",
    "geometries": "geojson",
    "overview": "full",
    "steps": "true",
  }

  resp = requests.get(OSRM_ROUTE_URL.format(coords=coords), params=params, timeout=10)
  resp.raise_for_status()
  data = resp.json()

  routes = data.get("routes") or []
  if not routes:
    return None, data

  route = routes[0]
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
