#!/usr/bin/env python3
"""Build a self-contained HTML report for a PX4 CP/VFH flight.

This is deliberately separate from analyze_vfh_ulog.py. It focuses on
reporting evidence from an existing ULog and does not change PX4 source code.
The report uses only pyulog, numpy and the Python standard library.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
from pyulog import ULog


MAX_POINTS = 7000
MAX_HEATMAP_ROWS = 1000
GAP_TIMEOUT_S = 0.7

PARAMETER_NAMES = (
    "CP_AUTO_EN",
    "CP_AUTO_DIST",
    "CP_AUTO_MODE",
    "CP_VFH_MAX_VEL",
    "CP_DIST",
    "CP_DELAY",
    "CP_MAX_ANGLE",
    "CP_BRAKE_ANGLE",
    "CP_VFH_STOP_DIST",
    "CP_STOP_VEL",
    "MPC_XY_CRUISE",
    "MPC_XY_VEL_MAX",
    "MPC_JERK_AUTO",
)

NAV_STATE_LABELS = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    10: "ACRO",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
}


def dataset_for(ulog: ULog, topic: str) -> Any | None:
    for dataset in ulog.data_list:
        if dataset.name == topic:
            return dataset
    return None


def field_for(dataset: Any | None, field: str) -> np.ndarray | None:
    if dataset is None or field not in dataset.data:
        return None
    return np.asarray(dataset.data[field])


def timestamp_for(dataset: Any | None) -> np.ndarray | None:
    timestamp = field_for(dataset, "timestamp")
    if timestamp is None:
        return None
    # Cast before subtracting. ULog timestamps are often uint64; diff on an
    # unsigned array can turn an ordinary gap into a huge wraparound value.
    return timestamp.astype(np.float64)


def finite_value(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def sample_indices(length: int, limit: int) -> np.ndarray:
    if length <= 0:
        return np.asarray([], dtype=np.int64)
    return np.unique(np.linspace(0, length - 1, min(length, limit), dtype=np.int64))


def as_series(timestamp: np.ndarray, values: np.ndarray, start_us: float,
              limit: int = MAX_POINTS, scale: float = 1.0) -> dict[str, Any]:
    length = min(len(timestamp), len(values))
    indices = sample_indices(length, limit)
    time_s = (timestamp[:length] - start_us) * 1e-6
    values = values[:length].astype(np.float64) * scale
    return {
        "t": [finite_value(time_s[index]) for index in indices],
        "v": [finite_value(values[index]) for index in indices],
    }


def field_series(dataset: Any | None, field: str, start_us: float,
                 scale: float = 1.0) -> dict[str, Any] | None:
    timestamp = timestamp_for(dataset)
    values = field_for(dataset, field)
    if timestamp is None or values is None:
        return None
    return as_series(timestamp, values, start_us, scale=scale)


def vector_series(dataset: Any | None, fields: tuple[str, ...], start_us: float,
                  limit: int = MAX_POINTS) -> dict[str, Any] | None:
    timestamp = timestamp_for(dataset)
    values = tuple(field_for(dataset, field) for field in fields)
    if timestamp is None or any(value is None for value in values):
        return None

    length = min(len(timestamp), *(len(value) for value in values if value is not None))
    indices = sample_indices(length, limit)
    time_s = (timestamp[:length] - start_us) * 1e-6
    arrays = tuple(value[:length].astype(np.float64) for value in values)
    return {
        "t": [finite_value(time_s[index]) for index in indices],
        "v": [[finite_value(array[index]) for array in arrays] for index in indices],
    }


def collect_start_time(datasets: list[Any | None]) -> float:
    starts = []
    for dataset in datasets:
        timestamp = timestamp_for(dataset)
        if timestamp is not None and len(timestamp):
            starts.append(float(timestamp[0]))
    return min(starts) if starts else 0.0


def parameters_from_ulog(ulog: ULog) -> tuple[dict[str, Any], dict[str, list[dict[str, Any]]]]:
    """Return final values and timestamped changes for the selected parameters."""
    initial = dict(getattr(ulog, "initial_parameters", {}) or {})
    changes: dict[str, list[dict[str, Any]]] = {name: [] for name in PARAMETER_NAMES}

    # pyulog versions differ here: initial_parameters is normally a dict, but
    # changed_parameters is a list of (timestamp, name, value) tuples.
    for record in getattr(ulog, "changed_parameters", []) or []:
        if not isinstance(record, (list, tuple)) or len(record) < 3:
            continue
        timestamp, name, value = record[:3]
        if name in changes:
            changes[name].append({
                "timestamp_us": float(timestamp),
                "value": finite_value(value) if isinstance(value, (int, float, np.number)) else value,
            })

    final = {name: initial.get(name) for name in PARAMETER_NAMES if name in initial}
    for name, records in changes.items():
        if records:
            final[name] = records[-1]["value"]

    final = {
        name: finite_value(value) if isinstance(value, (int, float, np.number)) else value
        for name, value in final.items()
        if value is not None
    }
    return final, {name: records for name, records in changes.items() if records}


def parameter_at(ulog: ULog, name: str, timestamp_us: float) -> float | None:
    initial = getattr(ulog, "initial_parameters", {}) or {}
    value = initial.get(name) if isinstance(initial, dict) else None
    for record in getattr(ulog, "changed_parameters", []) or []:
        if not isinstance(record, (list, tuple)) or len(record) < 3:
            continue
        changed_at, changed_name, changed_value = record[:3]
        if changed_name == name and float(changed_at) <= timestamp_us:
            value = changed_value
    return finite_value(value)


def parse_pose(element: ET.Element | None) -> tuple[float, float, float, float, float, float]:
    if element is None or element.text is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in element.text.split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])  # x, y, z, roll, pitch, yaw


def compose_pose(parent: tuple[float, float, float, float, float, float],
                 child: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float, float, float]:
    """Compose the simple SDF poses used by the obstacle course."""
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    return (
        px + cos_yaw * cx - sin_yaw * cy,
        py + sin_yaw * cx + cos_yaw * cy,
        pz + cz,
        croll,
        cpitch,
        pyaw + cyaw,
    )


def parse_world(world_path: Path | None) -> dict[str, Any]:
    """Extract static boxes and cylinders from the selected Gazebo SDF."""
    default_world = Path(__file__).resolve().parents[2] / "Tools/simulation/gz/worlds/vfh_obstacle_course.sdf"
    path = world_path or default_world
    result: dict[str, Any] = {
        "path": str(path),
        "coordinate_system": "Gazebo ENU: x=East, y=North, z=Up",
        "px4_transform": "PX4 NED (x=North,y=East,z=Down) -> Gazebo ENU (x=y_ned,y=x_ned,z=-z_ned)",
        "obstacles": [],
    }
    if not path.exists():
        result["error"] = "World file does not exist"
        return result

    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError, ValueError) as error:
        result["error"] = f"Could not parse SDF: {error}"
        return result

    for model in root.iter("model"):
        name = model.attrib.get("name", "model")
        if name == "ground_plane":
            continue
        model_pose = parse_pose(model.find("pose"))
        links = model.findall("link")
        if not links:
            continue
        for link in links:
            link_pose = compose_pose(model_pose, parse_pose(link.find("pose")))
            collisions = link.findall("collision") or link.findall("visual")
            for collision in collisions:
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                pose = compose_pose(link_pose, parse_pose(collision.find("pose")))
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None and box.find("size") is not None:
                    size = [float(value) for value in box.find("size").text.split()]
                    if len(size) >= 3:
                        result["obstacles"].append({
                            "name": name,
                            "type": "box",
                            "center": list(pose[:3]),
                            "size": size[:3],
                            "yaw_rad": pose[5],
                        })
                        break
                elif cylinder is not None:
                    radius = cylinder.find("radius")
                    length = cylinder.find("length")
                    if radius is not None and length is not None:
                        result["obstacles"].append({
                            "name": name,
                            "type": "cylinder",
                            "center": list(pose[:3]),
                            "radius": float(radius.text),
                            "height": float(length.text),
                            "yaw_rad": pose[5],
                        })
                        break
            else:
                continue
            break

    result["obstacle_count"] = len(result["obstacles"])
    return result


def paired_trajectory(position: Any | None, start_us: float) -> dict[str, Any] | None:
    fields = ("x", "y", "z", "vx", "vy", "vz")
    timestamp = timestamp_for(position)
    values = tuple(field_for(position, field) for field in fields)
    if timestamp is None or any(value is None for value in values):
        return None

    length = min(len(timestamp), *(len(value) for value in values if value is not None))
    indices = sample_indices(length, MAX_POINTS)
    arrays = tuple(value[:length].astype(np.float64) for value in values)
    speed = np.hypot(arrays[3], arrays[4])
    time_s = (timestamp[:length] - start_us) * 1e-6
    return {
        "t": [finite_value(time_s[index]) for index in indices],
        "x": [finite_value(arrays[0][index]) for index in indices],
        "y": [finite_value(arrays[1][index]) for index in indices],
        "z": [finite_value(arrays[2][index]) for index in indices],
        "vx": [finite_value(arrays[3][index]) for index in indices],
        "vy": [finite_value(arrays[4][index]) for index in indices],
        "vz": [finite_value(arrays[5][index]) for index in indices],
        "speed": [finite_value(speed[index]) for index in indices],
    }


def attitude_series(attitude: Any | None, start_us: float) -> dict[str, Any] | None:
    timestamp = timestamp_for(attitude)
    quaternion = tuple(field_for(attitude, f"q[{index}]") for index in range(4))
    if timestamp is None or any(value is None for value in quaternion):
        return None

    length = min(len(timestamp), *(len(value) for value in quaternion if value is not None))
    indices = sample_indices(length, MAX_POINTS)
    q = tuple(value[:length].astype(np.float64) for value in quaternion)
    yaw = np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                     1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
    roll = np.arctan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                      1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))
    pitch = np.arcsin(np.clip(2.0 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0))
    time_s = (timestamp[:length] - start_us) * 1e-6
    return {
        "t": [finite_value(time_s[index]) for index in indices],
        "roll_deg": [finite_value(math.degrees(roll[index])) for index in indices],
        "pitch_deg": [finite_value(math.degrees(pitch[index])) for index in indices],
        "yaw_deg": [finite_value(math.degrees(yaw[index])) for index in indices],
    }


def obstacle_data(dataset: Any | None, start_us: float) -> dict[str, Any] | None:
    timestamp = timestamp_for(dataset)
    distances = tuple(field_for(dataset, f"distances[{index}]") for index in range(72))
    max_distance = field_for(dataset, "max_distance")
    increment = field_for(dataset, "increment")
    angle_offset = field_for(dataset, "angle_offset")
    if timestamp is None or any(value is None for value in distances):
        return None

    length = min(len(timestamp), *(len(value) for value in distances if value is not None))
    if max_distance is not None:
        length = min(length, len(max_distance))
    raw = np.column_stack([value[:length].astype(np.float64) for value in distances])
    valid = np.isfinite(raw) & (raw < 65535.0)
    if max_distance is not None:
        valid &= raw <= max_distance[:length].astype(np.float64)[:, None]

    filled = np.where(valid, raw, np.inf)
    nearest_cm = np.min(filled, axis=1)
    has_valid = np.any(valid, axis=1)
    nearest_cm[~has_valid] = np.nan
    valid_count = np.sum(valid, axis=1)
    time_s = (timestamp[:length] - start_us) * 1e-6
    indices = sample_indices(length, MAX_POINTS)
    heat_indices = sample_indices(length, MAX_HEATMAP_ROWS)

    # Build an explicit health trace. A missing timestamp interval is not
    # represented by a uORB sample, so insert zero-valued points at both ends
    # of the interval instead of letting the plot connect valid samples across
    # the outage.
    status_time = []
    status_value = []
    for index in range(length):
        if index > 0 and time_s[index] - time_s[index - 1] > GAP_TIMEOUT_S:
            status_time.extend((time_s[index - 1], time_s[index]))
            status_value.extend((0.0, 0.0))
        status_time.append(time_s[index])
        status_value.append(1.0 if has_valid[index] else 0.0)
    status_indices = sample_indices(len(status_time), MAX_POINTS)

    max_values = None
    if max_distance is not None:
        max_values = [finite_value(max_distance[index] * 0.01) for index in indices]

    sectors = []
    for index in heat_indices:
        sectors.append([
            finite_value(raw[index, sector] * 0.01) if valid[index, sector] else None
            for sector in range(72)
        ])

    first_increment = finite_value(increment[0]) if increment is not None and len(increment) else None
    first_offset = finite_value(angle_offset[0]) if angle_offset is not None and len(angle_offset) else None
    return {
        "t": [finite_value(time_s[index]) for index in indices],
        "clearance": [finite_value(nearest_cm[index] * 0.01) for index in indices],
        "valid_count": [finite_value(valid_count[index]) for index in indices],
        "valid_fraction": [finite_value(valid_count[index] / 72.0) for index in indices],
        "sensor_status": {
            "t": [finite_value(status_time[index]) for index in status_indices],
            "v": [finite_value(status_value[index]) for index in status_indices],
        },
        "max_distance": max_values,
        "heatmap_t": [finite_value(time_s[index]) for index in heat_indices],
        "heatmap": sectors,
        "increment_deg": first_increment,
        "angle_offset_deg": first_offset,
        "source_topic": dataset.name,
        "raw_timestamp_us": timestamp.tolist(),
        "raw_clearance_m": [finite_value(value * 0.01) for value in nearest_cm],
        "raw_valid_count": valid_count.astype(np.int16).tolist(),
    }


def mode_data(status: Any | None, control_mode: Any | None, start_us: float) -> dict[str, Any] | None:
    timestamp = timestamp_for(status)
    nav_state = field_for(status, "nav_state")
    if timestamp is None or nav_state is None:
        return None

    length = min(len(timestamp), len(nav_state))
    indices = sample_indices(length, MAX_POINTS)
    time_s = (timestamp[:length] - start_us) * 1e-6
    result: dict[str, Any] = {
        "t": [finite_value(time_s[index]) for index in indices],
        "nav_state": [int(nav_state[index]) for index in indices],
        "nav_label": [NAV_STATE_LABELS.get(int(nav_state[index]), str(int(nav_state[index]))) for index in indices],
    }

    if control_mode is not None:
        control_timestamp = timestamp_for(control_mode)
        manual = field_for(control_mode, "flag_control_manual_enabled")
        auto = field_for(control_mode, "flag_control_auto_enabled")
        if control_timestamp is not None and manual is not None and auto is not None:
            result["control_mode"] = {
                "t": [finite_value((value - start_us) * 1e-6) for value in control_timestamp],
                "manual_enabled": [int(value) for value in manual],
                "auto_enabled": [int(value) for value in auto],
            }
    return result


def command_events(command: Any | None, start_us: float) -> list[dict[str, Any]]:
    timestamp = timestamp_for(command)
    command_id = field_for(command, "command")
    param2 = field_for(command, "param2")
    param3 = field_for(command, "param3")
    source_system = field_for(command, "source_system")
    source_component = field_for(command, "source_component")
    if timestamp is None or command_id is None:
        return []

    length = min(len(timestamp), len(command_id))
    events = []
    for index in range(length):
        if int(command_id[index]) != 176:
            continue
        p2 = float(param2[index]) if param2 is not None and index < len(param2) else math.nan
        p3 = float(param3[index]) if param3 is not None and index < len(param3) else math.nan
        if math.isclose(p2, 3.0) and math.isclose(p3, 0.0):
            mode = "POSCTL"
            severity = "critical"
        elif math.isclose(p2, 4.0) and math.isclose(p3, 3.0):
            mode = "AUTO_LOITER"
            severity = "warning"
        else:
            mode = f"DO_SET_MODE p2={p2:g} p3={p3:g}"
            severity = "info"

        internal = (source_system is not None and source_component is not None
                    and int(source_system[index]) == 1 and int(source_component[index]) == 1)
        origin = "autopilot/internal" if internal else "external/QGC/other"
        events.append({
            "time": float((timestamp[index] - start_us) * 1e-6),
            "severity": severity,
            "kind": "mode_command",
            "message": f"{origin} requested {mode}",
            "mode": mode,
            "internal": internal,
            "duration_s": 0.0,
        })
    return events


def status_transition_events(status: Any | None, start_us: float) -> list[dict[str, Any]]:
    timestamp = timestamp_for(status)
    nav_state = field_for(status, "nav_state")
    if timestamp is None or nav_state is None or not len(nav_state):
        return []
    transitions = np.r_[0, np.where(nav_state[1:] != nav_state[:-1])[0] + 1]
    events = []
    for index in transitions[1:]:
        label = NAV_STATE_LABELS.get(int(nav_state[index]), str(int(nav_state[index])))
        events.append({
            "time": float((timestamp[index] - start_us) * 1e-6),
            "severity": "info",
            "kind": "mode_transition",
            "message": f"vehicle_status entered {label} (nav_state={int(nav_state[index])})",
            "duration_s": 0.0,
        })
    return events


def sensor_gap_events(obstacle: Any | None, start_us: float,
                      timeout_s: float = GAP_TIMEOUT_S) -> list[dict[str, Any]]:
    timestamp = timestamp_for(obstacle)
    if timestamp is None or len(timestamp) < 2:
        return []
    delta = np.diff(timestamp) * 1e-6
    events = []
    for index in np.where(delta > timeout_s)[0]:
        start = float((timestamp[index] - start_us) * 1e-6)
        end = float((timestamp[index + 1] - start_us) * 1e-6)
        events.append({
            "time": start,
            "severity": "critical",
            "kind": "sensor_gap",
            "message": f"{obstacle.name} gap: no sample for {delta[index]:.2f} s",
            "duration_s": float(delta[index]),
            "end_time": end,
        })
    return events


def boolean_interval_events(timestamp: np.ndarray, active: np.ndarray,
                             start_us: float, severity: str, kind: str,
                             message: str) -> list[dict[str, Any]]:
    length = min(len(timestamp), len(active))
    if length == 0:
        return []
    active = active[:length].astype(bool)
    timestamp = timestamp[:length]
    events = []
    begin = None
    for index, value in enumerate(np.r_[active, False]):
        if value and begin is None:
            begin = index
        elif not value and begin is not None:
            end = index - 1
            events.append({
                "time": float((timestamp[begin] - start_us) * 1e-6),
                "severity": severity,
                "kind": kind,
                "message": message,
                "duration_s": float(max(0.0, (timestamp[end] - timestamp[begin]) * 1e-6)),
            })
            begin = None
    return events


def threshold_events(ulog: ULog, obstacle: Any | None, start_us: float,
                     parameter: str, kind: str, severity: str) -> list[dict[str, Any]]:
    if obstacle is None:
        return []
    timestamp = timestamp_for(obstacle)
    distances = tuple(field_for(obstacle, f"distances[{index}]") for index in range(72))
    max_distance = field_for(obstacle, "max_distance")
    if timestamp is None or any(value is None for value in distances):
        return []
    length = min(len(timestamp), *(len(value) for value in distances if value is not None))
    if max_distance is not None:
        length = min(length, len(max_distance))
    raw = np.column_stack([value[:length].astype(np.float64) for value in distances])
    valid = np.isfinite(raw) & (raw < 65535.0)
    if max_distance is not None:
        valid &= raw <= max_distance[:length].astype(np.float64)[:, None]
    nearest = np.min(np.where(valid, raw, np.inf), axis=1) * 0.01
    nearest[~np.any(valid, axis=1)] = np.nan
    thresholds = np.asarray([
        parameter_at(ulog, parameter, value) or math.nan for value in timestamp[:length]
    ])
    active = np.isfinite(nearest) & np.isfinite(thresholds) & (thresholds > 0) & (nearest < thresholds)
    return boolean_interval_events(
        timestamp[:length], active, start_us, severity, kind,
        f"Obstacle clearance below time-varying {parameter}",
    )


def cp_intervention_events(constraints: Any | None, start_us: float) -> list[dict[str, Any]]:
    timestamp = timestamp_for(constraints)
    original_x = field_for(constraints, "original_setpoint[0]")
    original_y = field_for(constraints, "original_setpoint[1]")
    adapted_x = field_for(constraints, "adapted_setpoint[0]")
    adapted_y = field_for(constraints, "adapted_setpoint[1]")
    if timestamp is None or any(value is None for value in (original_x, original_y, adapted_x, adapted_y)):
        return []
    length = min(len(timestamp), len(original_x), len(original_y), len(adapted_x), len(adapted_y))
    delta = np.hypot(original_x[:length] - adapted_x[:length], original_y[:length] - adapted_y[:length])
    active = np.isfinite(delta) & (delta > 0.01)
    return boolean_interval_events(
        timestamp[:length], active, start_us, "warning", "cp_intervention",
        "Collision Prevention changed the logged XY setpoint",
    )


def metrics(ulog: ULog, datasets: dict[str, Any | None], obstacle_report: dict[str, Any] | None,
            trajectory: dict[str, Any] | None, commands: list[dict[str, Any]],
            events: list[dict[str, Any]], start_us: float) -> dict[str, Any]:
    actual_speed = trajectory.get("speed", []) if trajectory else []
    speed_values = [value for value in actual_speed if value is not None]
    clearance_values = obstacle_report.get("raw_clearance_m", []) if obstacle_report else []
    clearance_values = [value for value in clearance_values if value is not None]
    position_status = [event for event in commands
                       if event.get("internal") and event.get("mode") == "POSCTL"]
    sensor_gaps = [event for event in events if event["kind"] == "sensor_gap"]

    position = datasets.get("vehicle_local_position")
    position_timestamp = timestamp_for(position)
    status = datasets.get("vehicle_status")
    status_timestamp = timestamp_for(status)
    nav_state = field_for(status, "nav_state")
    auto_mask = np.zeros(0, dtype=bool)
    if position_timestamp is not None and status_timestamp is not None and nav_state is not None:
        state_at_position = np.interp(position_timestamp, status_timestamp, nav_state)
        auto_mask = np.isclose(state_at_position, 3.0)
    auto_speed_max = None
    if auto_mask.any() and position is not None:
        vx = field_for(position, "vx")
        vy = field_for(position, "vy")
        if vx is not None and vy is not None:
            auto_speed_max = finite_value(np.nanmax(np.hypot(vx[auto_mask], vy[auto_mask])))

    return {
        "duration_s": finite_value((ulog.last_timestamp - start_us) / 1e6),
        "max_actual_speed_m_s": finite_value(max(speed_values)) if speed_values else None,
        "max_auto_mission_speed_m_s": auto_speed_max,
        "min_logged_clearance_m": finite_value(min(clearance_values)) if clearance_values else None,
        "cp_intervention_events": sum(event["kind"] == "cp_intervention" for event in events),
        "sensor_gap_events": len(sensor_gaps),
        "internal_posctl_requests": len(position_status),
    }


def build_report(path: Path, world_path: Path | None = None) -> dict[str, Any]:
    ulog = ULog(str(path))
    topic_names = (
        "vehicle_local_position",
        "vehicle_local_position_setpoint",
        "vehicle_attitude",
        "vehicle_acceleration",
        "obstacle_distance",
        "obstacle_distance_fused",
        "collision_constraints",
        "manual_control_setpoint",
        "vehicle_status",
        "vehicle_control_mode",
        "vehicle_command",
        "position_setpoint_triplet",
    )
    datasets = {topic: dataset_for(ulog, topic) for topic in topic_names}
    start_us = float(getattr(ulog, "start_timestamp", 0) or collect_start_time(list(datasets.values())))

    # Prefer raw obstacle_distance for gap timing. The fused topic can contain
    # zero-timestamp placeholder samples when the source changes.
    obstacle = datasets["obstacle_distance"] or datasets["obstacle_distance_fused"]
    position = datasets["vehicle_local_position"]
    position_setpoint = datasets["vehicle_local_position_setpoint"]
    constraints = datasets["collision_constraints"]
    attitude = datasets["vehicle_attitude"]

    trajectory = paired_trajectory(position, start_us)
    obstacle_report = obstacle_data(obstacle, start_us)
    original = vector_series(constraints, ("original_setpoint[0]", "original_setpoint[1]"), start_us)
    adapted = vector_series(constraints, ("adapted_setpoint[0]", "adapted_setpoint[1]"), start_us)
    if original and adapted:
        original["magnitude"] = [
            finite_value(math.hypot(value[0], value[1])) if value[0] is not None and value[1] is not None else None
            for value in original["v"]
        ]
        adapted["magnitude"] = [
            finite_value(math.hypot(value[0], value[1])) if value[0] is not None and value[1] is not None else None
            for value in adapted["v"]
        ]

    params, parameter_changes = parameters_from_ulog(ulog)
    world = parse_world(world_path)
    modes = mode_data(datasets["vehicle_status"], datasets["vehicle_control_mode"], start_us)
    commands = command_events(datasets["vehicle_command"], start_us)
    events = status_transition_events(datasets["vehicle_status"], start_us)
    events.extend(commands)
    events.extend(sensor_gap_events(obstacle, start_us))
    events.extend(cp_intervention_events(constraints, start_us))
    events.extend(threshold_events(ulog, obstacle, start_us, "CP_DIST", "clearance_threshold", "warning"))

    # Speed and angle checks are observations of the flight, not proof that a
    # specific internal VFH branch was taken.
    if trajectory:
        t = np.asarray([value * 1e6 + start_us for value in trajectory["t"]], dtype=np.float64)
        speed = np.asarray([value if value is not None else np.nan for value in trajectory["speed"]])
        cap = parameter_at(ulog, "CP_VFH_MAX_VEL", start_us) or 3.0
        events.extend(boolean_interval_events(
            t, np.isfinite(speed) & (speed > cap * 1.05), start_us,
            "warning", "speed_limit", f"Actual horizontal speed exceeded CP_VFH_MAX_VEL={cap:.2f} m/s",
        ))

    events.sort(key=lambda event: event["time"])
    parameter_changes_for_json = {
        name: [
            {"time": finite_value((record["timestamp_us"] - start_us) / 1e6), "value": record["value"]}
            for record in records
        ]
        for name, records in parameter_changes.items()
    }

    report = {
        "meta": {
            "ulog": str(path),
            "start_timestamp_us": start_us,
            "duration_s": finite_value((ulog.last_timestamp - start_us) / 1e6),
            "topics": {topic: datasets[topic] is not None for topic in topic_names},
        },
        "params": params,
        "parameter_changes": parameter_changes_for_json,
        "trajectory": trajectory,
        "position_setpoint": vector_series(position_setpoint, ("x", "y", "z"), start_us),
        "velocity_setpoint": vector_series(position_setpoint, ("vx", "vy", "vz"), start_us),
        "setpoint_acceleration": vector_series(position_setpoint, ("acceleration[0]", "acceleration[1]", "acceleration[2]"), start_us),
        "manual_control": vector_series(datasets["manual_control_setpoint"], ("roll", "pitch", "yaw", "throttle"), start_us),
        "manual_sticks_moving": field_series(datasets["manual_control_setpoint"], "sticks_moving", start_us),
        "vehicle_acceleration_frd": vector_series(datasets["vehicle_acceleration"], ("xyz[0]", "xyz[1]", "xyz[2]"), start_us),
        "attitude": attitude_series(attitude, start_us),
        "collision_constraints": {"original": original, "adapted": adapted},
        "obstacle": obstacle_report,
        "world": world,
        "modes": modes,
        "commands": commands,
        "events": events,
        "summary": metrics(ulog, datasets, obstacle_report, trajectory, commands, events, start_us),
        "notes": [
            "Observed: vehicle_status, vehicle_command, obstacle_distance, manual_control_setpoint and collision_constraints are logged topics.",
            "Computed: clearance, speed, CP setpoint delta, body-frame acceleration and sensor gaps are calculated from logged samples.",
            "Obstacle data status is 1 when at least one sector is valid and 0 for invalid samples or an inserted timestamp-gap interval.",
            "Inferred: exact VFH selected direction, cost, free sector count and internal critical/braking flags are not logged as uORB fields.",
            "vehicle_acceleration.xyz is FRD body-frame acceleration and includes gravity; the horizontal X/Y channels are used for the report.",
            "In the current manual StickAccelerationXY path, collision_constraints contains acceleration setpoints even though the legacy message comment says velocities; the chart is therefore labelled as a setpoint trace, not as a velocity measurement.",
            "POSCTL and AUTO_LOITER command markers are evidence of mode requests; the report keeps the originating command and the resulting vehicle_status transition separate.",
        ],
    }
    return report


def html_report(report: dict[str, Any], title: str, plot_scale: float = 3.0) -> str:
    payload = json.dumps(report, allow_nan=False, separators=(",", ":"))
    payload = payload.replace("<", "\\u003c")
    safe_title = html.escape(title)
    plot_scale = min(5.0, max(1.0, float(plot_scale)))
    # A full-width row gives the time axis the same readable shape as Flight
    # Review. The scale controls horizontal report width, not vertical height.
    report_width = 480.0 * plot_scale
    chart_height = 320.0
    large_chart_height = 460.0
    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{safe_title}</title>
<style>
:root {{ color-scheme: dark; font-family: system-ui, sans-serif; }}
body {{ margin: 0; background: #101418; color: #e8edf2; }}
main {{ max-width: {report_width:.0f}px; margin: auto; padding: 22px; }}
h1 {{ margin: 0 0 12px; font-size: 1.5rem; }}
h2 {{ margin: 0 0 9px; font-size: 1rem; }}
.muted {{ color: #aeb9c3; font-size: .88rem; }}
.summary, .plots {{ display: grid; grid-template-columns: 1fr; gap: 10px; }}
.summary {{ grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); margin: 12px 0; }}
.card {{ background: #192027; border: 1px solid #303b45; border-radius: 6px; padding: 12px; }}
.wide {{ grid-column: 1 / -1; }}
canvas {{ display: block; width: 100%; height: {chart_height:.0f}px; background: #12181d; border-radius: 4px; }}
.trajectory, .heatmap {{ height: {large_chart_height:.0f}px; }}
.scene3d, .trajectory {{ cursor: grab; }}
.scene3d.dragging, .trajectory.dragging {{ cursor: grabbing; }}
.plot-heading {{ display: flex; align-items: center; justify-content: space-between; gap: 10px; }}
.reset-view {{ border: 1px solid #516170; border-radius: 4px; background: #202a33; color: #e8edf2; padding: 4px 9px; cursor: pointer; }}
.reset-view:hover {{ background: #2c3a46; }}
table {{ width: 100%; border-collapse: collapse; font-size: .86rem; }}
th, td {{ text-align: left; padding: 6px; border-bottom: 1px solid #303b45; vertical-align: top; }}
.critical {{ color: #ff7b72; }} .warning {{ color: #f2cc60; }} .info {{ color: #9cdcfe; }}
code {{ color: #9cdcfe; }}
</style>
</head>
<body>
<main>
<h1>{safe_title}</h1>
<div id="summary" class="summary"></div>
<section class="card">
  <h2>Reading guide</h2>
  <p class="muted">Use the mode timeline together with the command markers. For CP behavior, compare joystick input or original setpoint against the adapted setpoint. For sensor loss, check the obstacle data-validity trace and the gap events. Exact VFH planner flags are unavailable unless a dedicated uORB debug topic is added.</p>
  <div id="notes" class="muted"></div>
</section>
<section class="card wide" style="margin-top:10px"><h2>Local trajectory (NED X/Y)</h2><canvas id="trajectory" class="trajectory"></canvas></section>
<section class="card wide" style="margin-top:10px"><h2>Gazebo world and 3D trajectory</h2><canvas id="scene3d" class="scene3d"></canvas><p id="world-info" class="muted"></p></section>
<section class="plots" style="margin-top:10px">
  <section class="card"><h2>Mode timeline</h2><canvas id="mode"></canvas></section>
  <section class="card"><h2>Horizontal velocity</h2><canvas id="velocity"></canvas></section>
  <section class="card"><h2>Joystick input</h2><canvas id="joystick"></canvas></section>
  <section class="card"><h2>CP original vs adapted XY setpoint</h2><canvas id="constraints"></canvas></section>
  <section class="card"><h2>Body acceleration FRD</h2><canvas id="acceleration"></canvas></section>
  <section class="card"><h2>Obstacle clearance</h2><canvas id="clearance"></canvas></section>
  <section class="card"><h2>Obstacle data validity</h2><canvas id="validity"></canvas></section>
  <section class="card"><h2>Obstacle data status (1 valid / 0 lost)</h2><canvas id="sensor_status"></canvas></section>
  <section class="card"><h2>Attitude</h2><canvas id="attitude"></canvas></section>
  <section class="card wide"><h2>72-sector obstacle heatmap</h2><canvas id="heatmap" class="heatmap"></canvas></section>
</section>
<section class="card wide" style="margin-top:10px"><h2>Detected events</h2><table><thead><tr><th>Interval</th><th>Level</th><th>Kind</th><th>Evidence</th><th>Duration</th></tr></thead><tbody id="events"></tbody></table></section>
<section class="plots" style="margin-top:10px">
  <section class="card"><h2>Logged topics</h2><table><thead><tr><th>Topic</th><th>Present</th></tr></thead><tbody id="topics"></tbody></table></section>
  <section class="card"><h2>Parameters</h2><table><thead><tr><th>Parameter</th><th>Final logged value</th><th>Changes</th></tr></thead><tbody id="params"></tbody></table></section>
</section>
</main>
<script>
const REPORT = {payload};

function pairs(t, v) {{
  const out=[]; if (!t || !v) return out;
  for (let i=0; i<Math.min(t.length,v.length); i++) if (v[i] !== null && Number.isFinite(v[i])) out.push([t[i],v[i]]);
  return out;
}}

const CHARTS = {{}};
const VIEWS = {{}};
const TRAJECTORY_VIEW = {{}};

function chartBounds(lines, horizontal) {{
  const all=lines.flatMap(line=>pairs(line.t,line.v));
  if (!all.length) return null;
  let xmin=Math.min(...all.map(p=>p[0])), xmax=Math.max(...all.map(p=>p[0]));
  let ymin=Math.min(...all.map(p=>p[1])), ymax=Math.max(...all.map(p=>p[1]));
  horizontal.forEach(line=>{{ymin=Math.min(ymin,line.value);ymax=Math.max(ymax,line.value);}});
  if (xmax-xmin<1e-6) {{xmin-=1;xmax+=1;}}
  if (ymax-ymin<1e-6) {{ymin-=1;ymax+=1;}}
  return {{xmin,xmax,ymin,ymax}};
}}

function redrawChart(id) {{
  const chart=CHARTS[id];
  if (chart) drawChart(id, chart.lines, chart.yLabel, chart.horizontal);
}}

function resetChart(id) {{
  delete VIEWS[id];
  redrawChart(id);
}}

function bindChartInteractions(canvas, id) {{
  if (canvas.dataset.interactive === 'true') return;
  canvas.dataset.interactive='true';
  canvas.addEventListener('wheel', event=>{{
    event.preventDefault();
    const view=VIEWS[id], rect=canvas.getBoundingClientRect(), width=Math.max(640,rect.width), height=Math.max(260,rect.height);
    if (!view) return;
    const pad={{l:58,r:18,t:22,b:35}}, px=(event.clientX-rect.left)/Math.max(1,rect.width)*width, py=(event.clientY-rect.top)/Math.max(1,rect.height)*height;
    const tx=view.xmin+(px-pad.l)/(width-pad.l-pad.r)*(view.xmax-view.xmin), ty=view.ymax-(py-pad.t)/(height-pad.t-pad.b)*(view.ymax-view.ymin);
    const factor=event.deltaY<0 ? 0.82 : 1.22;
    let xspan=(view.xmax-view.xmin)*factor, yspan=(view.ymax-view.ymin)*factor;
    xspan=Math.max(1e-4,Math.min(xspan,1e9)); yspan=Math.max(1e-4,Math.min(yspan,1e9));
    view.xmin=tx-(tx-view.xmin)*factor; view.xmax=view.xmin+xspan;
    view.ymin=ty-(ty-view.ymin)*factor; view.ymax=view.ymin+yspan;
    redrawChart(id);
  }},{{passive:false}});
  canvas.addEventListener('pointerdown', event=>{{
    canvas.setPointerCapture(event.pointerId);
    canvas._drag={{x:event.clientX,y:event.clientY}};
    canvas.classList.add('dragging');
  }});
  canvas.addEventListener('pointermove', event=>{{
    if (!canvas._drag) return;
    const view=VIEWS[id], rect=canvas.getBoundingClientRect(), width=Math.max(640,rect.width), height=Math.max(260,rect.height), pad={{l:58,r:18,t:22,b:35}};
    const dx=(event.clientX-canvas._drag.x)/Math.max(1,rect.width)*(view.xmax-view.xmin);
    const dy=(event.clientY-canvas._drag.y)/Math.max(1,rect.height)*(view.ymax-view.ymin);
    view.xmin-=dx;view.xmax-=dx;view.ymin+=dy;view.ymax+=dy;canvas._drag={{x:event.clientX,y:event.clientY}};
    redrawChart(id);
  }});
  const stopDrag=event=>{{if(canvas._drag){{canvas._drag=null;canvas.classList.remove('dragging');if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);}}}};
  canvas.addEventListener('pointerup',stopDrag);canvas.addEventListener('pointercancel',stopDrag);canvas.addEventListener('dblclick',()=>resetChart(id));canvas.addEventListener('contextmenu',event=>event.preventDefault());
}}

function drawChart(id, lines, yLabel, horizontal=[]) {{
  if (id === 'constraints') yLabel = 'logged XY setpoint (manual path: m/s2)';
  CHARTS[id]={{lines,yLabel,horizontal}};
  const canvas=document.getElementById(id), rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  const width=Math.max(640,rect.width), height=Math.max(260,rect.height); canvas.width=width*dpr; canvas.height=height*dpr;
  const ctx=canvas.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0); ctx.fillStyle='#12181d'; ctx.fillRect(0,0,width,height);
  const base=chartBounds(lines,horizontal);
  if (!base) {{ ctx.fillStyle='#aeb9c3'; ctx.fillText('Required topic or field is not available',20,28); bindChartInteractions(canvas,id); return; }}
  if (!VIEWS[id]) VIEWS[id]={{...base}};
  const view=VIEWS[id], xmin=view.xmin, xmax=view.xmax, ymin=view.ymin, ymax=view.ymax;
  const pad={{l:58,r:18,t:22,b:35}}, X=t=>pad.l+(t-xmin)/(xmax-xmin)*(width-pad.l-pad.r), Y=v=>height-pad.b-(v-ymin)/(ymax-ymin)*(height-pad.t-pad.b);
  ctx.strokeStyle='#303b45';ctx.lineWidth=1;ctx.fillStyle='#aeb9c3';ctx.font='11px system-ui';
  for(let i=0;i<=5;i++){{const yy=pad.t+i*(height-pad.t-pad.b)/5;ctx.beginPath();ctx.moveTo(pad.l,yy);ctx.lineTo(width-pad.r,yy);ctx.stroke();ctx.fillText((ymax-(ymax-ymin)*i/5).toFixed(2),5,yy+4);}}
  ctx.fillText('time [s]',width/2-20,height-8);ctx.save();ctx.translate(12,height/2+20);ctx.rotate(-Math.PI/2);ctx.fillText(yLabel,0,0);ctx.restore();
  horizontal.forEach(line=>{{ctx.strokeStyle=line.color||'#f2cc60';ctx.setLineDash([5,4]);ctx.beginPath();ctx.moveTo(pad.l,Y(line.value));ctx.lineTo(width-pad.r,Y(line.value));ctx.stroke();ctx.setLineDash([]);ctx.fillStyle=line.color||'#f2cc60';ctx.fillText(line.label||String(line.value),pad.l+5,Y(line.value)-4);}});
  lines.forEach(line=>{{const ps=pairs(line.t,line.v);if(!ps.length)return;ctx.strokeStyle=line.color;ctx.lineWidth=1.7;ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke();}});
  let lx=pad.l;lines.forEach(line=>{{ctx.fillStyle=line.color;ctx.fillRect(lx,7,13,3);ctx.fillText(line.name,lx+17,11);lx+=Math.max(88,line.name.length*7+32);}});
  bindChartInteractions(canvas,id);
}}

function trajectoryBounds(actual, target) {{
  const all=actual.concat(target);
  if (!all.length) return null;
  let xmin=Math.min(...all.map(p=>p[0])), xmax=Math.max(...all.map(p=>p[0]));
  let ymin=Math.min(...all.map(p=>p[1])), ymax=Math.max(...all.map(p=>p[1]));
  if (xmax-xmin<1) {{xmin-=1;xmax+=1;}}
  if (ymax-ymin<1) {{ymin-=1;ymax+=1;}}
  return {{xmin,xmax,ymin,ymax}};
}}

function resetTrajectory() {{ delete TRAJECTORY_VIEW.xmin; drawTrajectory(); }}

function bindTrajectoryInteractions(canvas) {{
  if (canvas.dataset.interactive === 'true') return;
  canvas.dataset.interactive='true';
  canvas.addEventListener('wheel', event=>{{
    event.preventDefault();
    const view=TRAJECTORY_VIEW, rect=canvas.getBoundingClientRect(), width=Math.max(640,rect.width), height=Math.max(360,rect.height);
    if (view.xmin === undefined) return;
    const pad={{l:55,r:18,t:22,b:38}}, px=(event.clientX-rect.left)/Math.max(1,rect.width)*width, py=(event.clientY-rect.top)/Math.max(1,rect.height)*height;
    const tx=view.xmin+(px-pad.l)/(width-pad.l-pad.r)*(view.xmax-view.xmin), ty=view.ymax-(py-pad.t)/(height-pad.t-pad.b)*(view.ymax-view.ymin);
    const factor=event.deltaY<0 ? 0.82 : 1.22;
    const xspan=(view.xmax-view.xmin)*factor, yspan=(view.ymax-view.ymin)*factor;
    view.xmin=tx-(tx-view.xmin)*factor; view.xmax=view.xmin+xspan;
    view.ymin=ty-(ty-view.ymin)*factor; view.ymax=view.ymin+yspan;
    drawTrajectory();
  }},{{passive:false}});
  canvas.addEventListener('pointerdown', event=>{{
    canvas.setPointerCapture(event.pointerId);
    canvas._drag={{x:event.clientX,y:event.clientY}};
    canvas.classList.add('dragging');
  }});
  canvas.addEventListener('pointermove', event=>{{
    if (!canvas._drag || TRAJECTORY_VIEW.xmin === undefined) return;
    const view=TRAJECTORY_VIEW, rect=canvas.getBoundingClientRect();
    const dx=(event.clientX-canvas._drag.x)/Math.max(1,rect.width)*(view.xmax-view.xmin);
    const dy=(event.clientY-canvas._drag.y)/Math.max(1,rect.height)*(view.ymax-view.ymin);
    view.xmin-=dx; view.xmax-=dx; view.ymin+=dy; view.ymax+=dy;
    canvas._drag={{x:event.clientX,y:event.clientY}};
    drawTrajectory();
  }});
  const stopDrag=event=>{{if(canvas._drag){{canvas._drag=null;canvas.classList.remove('dragging');if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);}}}};
  canvas.addEventListener('pointerup',stopDrag); canvas.addEventListener('pointercancel',stopDrag);
  canvas.addEventListener('dblclick',resetTrajectory); canvas.addEventListener('contextmenu',event=>event.preventDefault());
}}

function drawTrajectory() {{
  const canvas=document.getElementById('trajectory'),rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(640,rect.width),h=Math.max(360,rect.height);canvas.width=w*dpr;canvas.height=h*dpr;const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#12181d';ctx.fillRect(0,0,w,h);
  const tr=REPORT.trajectory,sp=REPORT.position_setpoint;if(!tr){{ctx.fillStyle='#aeb9c3';ctx.fillText('vehicle_local_position is not available',20,28);bindTrajectoryInteractions(canvas);return;}}
  const actual=tr.x.map((x,i)=>x!==null&&tr.y[i]!==null?[x,tr.y[i]]:null).filter(Boolean),target=sp?sp.v.map(v=>v&&v[0]!==null&&v[1]!==null?[v[0],v[1]]:null).filter(Boolean):[],base=trajectoryBounds(actual,target);if(!base){{ctx.fillStyle='#aeb9c3';ctx.fillText('No valid trajectory samples',20,28);bindTrajectoryInteractions(canvas);return;}}if(TRAJECTORY_VIEW.xmin===undefined)Object.assign(TRAJECTORY_VIEW,base);const view=TRAJECTORY_VIEW,xmin=view.xmin,xmax=view.xmax,ymin=view.ymin,ymax=view.ymax;const pad={{l:55,r:18,t:22,b:38}},X=x=>pad.l+(x-xmin)/(xmax-xmin)*(w-pad.l-pad.r),Y=y=>h-pad.b-(y-ymin)/(ymax-ymin)*(h-pad.t-pad.b);ctx.strokeStyle='#303b45';ctx.lineWidth=1;for(let i=0;i<=5;i++){{const xx=pad.l+i*(w-pad.l-pad.r)/5,yy=pad.t+i*(h-pad.t-pad.b)/5;ctx.beginPath();ctx.moveTo(xx,pad.t);ctx.lineTo(xx,h-pad.b);ctx.moveTo(pad.l,yy);ctx.lineTo(w-pad.r,yy);ctx.stroke();}}function line(data,color,dash){{ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dash||[]);ctx.beginPath();data.forEach((p,i)=>i?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke();ctx.setLineDash([]);}}line(actual,'#4fc3f7');if(target.length)line(target,'#f2cc60',[6,4]);ctx.fillStyle='#aeb9c3';ctx.fillText('NED X [m]',w/2-24,h-10);ctx.save();ctx.translate(14,h/2+15);ctx.rotate(-Math.PI/2);ctx.fillText('NED Y [m]',0,0);ctx.restore();ctx.fillStyle='#4fc3f7';ctx.fillRect(pad.l,8,14,3);ctx.fillText('actual',pad.l+18,12);ctx.fillStyle='#f2cc60';ctx.fillRect(pad.l+83,8,14,3);ctx.fillText('position setpoint',pad.l+103,12);bindTrajectoryInteractions(canvas);
}}

function drawMode() {{
  const canvas=document.getElementById('mode'),rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(640,rect.width),h=Math.max(260,rect.height);canvas.width=w*dpr;canvas.height=h*dpr;const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#12181d';ctx.fillRect(0,0,w,h);const m=REPORT.modes;if(!m){{ctx.fillStyle='#aeb9c3';ctx.fillText('vehicle_status is not available',20,28);return;}}const t=m.t,v=m.nav_state,valid=t.length;if(!valid)return;const xmin=t[0],xmax=t[t.length-1],pad={{l:110,r:18,t:22,b:35}},X=q=>pad.l+(q-xmin)/Math.max(1e-6,xmax-xmin)*(w-pad.l-pad.r),labels={{0:'MANUAL',1:'ALTCTL',2:'POSCTL',3:'AUTO_MISSION',4:'AUTO_LOITER',5:'AUTO_RTL',14:'OFFBOARD',15:'STAB'}};const values=[0,2,3,4,5,14,15];const Y=q=>h-pad.b-(values.indexOf(q)<0?0:values.indexOf(q))/(values.length-1)*(h-pad.t-pad.b);ctx.font='11px system-ui';values.forEach(q=>{{ctx.fillStyle='#aeb9c3';ctx.fillText(labels[q]||String(q),5,Y(q)+4);ctx.strokeStyle='#303b45';ctx.beginPath();ctx.moveTo(pad.l,Y(q));ctx.lineTo(w-pad.r,Y(q));ctx.stroke();}});ctx.strokeStyle='#4fc3f7';ctx.lineWidth=2;ctx.beginPath();v.forEach((q,i)=>{{const x=X(t[i]),y=Y(q);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}});ctx.stroke();const cm=m.control_mode;if(cm){{function flagLine(field,color,y){{ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();let started=false;for(let i=0;i<cm.t.length;i++){{if(!cm[field][i]){{started=false;continue;}}const x=X(cm.t[i]);if(!started){{ctx.moveTo(x,y);started=true;}}else{{ctx.lineTo(x,y);}}}}ctx.stroke();}}flagLine('manual_enabled','#f2cc60',h-pad.b-10);flagLine('auto_enabled','#ff7b72',h-pad.b-22);}}ctx.fillStyle='#aeb9c3';ctx.fillText('time [s]',w/2-20,h-8);ctx.fillStyle='#4fc3f7';ctx.fillRect(pad.l,7,14,3);ctx.fillText('vehicle_status',pad.l+18,11);const cmLegend=m.control_mode;if(cmLegend){{ctx.fillStyle='#f2cc60';ctx.fillRect(pad.l+125,7,14,3);ctx.fillText('manual_enabled',pad.l+143,11);ctx.fillStyle='#ff7b72';ctx.fillRect(pad.l+245,7,14,3);ctx.fillText('auto_enabled',pad.l+263,11);}}
}}

function drawHeatmap() {{
  const canvas=document.getElementById('heatmap'),rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=Math.max(640,rect.width),h=Math.max(360,rect.height);canvas.width=w*dpr;canvas.height=h*dpr;const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#12181d';ctx.fillRect(0,0,w,h);const o=REPORT.obstacle;if(!o||!o.heatmap||!o.heatmap.length){{ctx.fillStyle='#aeb9c3';ctx.fillText('obstacle_distance is not available',20,28);return;}}const rows=o.heatmap.length,cols=72,pad={{l:58,r:18,t:24,b:35}},cw=(w-pad.l-pad.r)/rows,ch=(h-pad.t-pad.b)/cols;for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){{const value=o.heatmap[r][c];if(value===null)ctx.fillStyle='#303840';else if(value<1.5)ctx.fillStyle='#ff5c5c';else if(value<3)ctx.fillStyle='#f2cc60';else{{const g=Math.max(40,Math.min(220,40+value*12));ctx.fillStyle=`rgb(40,${{g}},110)`;}}ctx.fillRect(pad.l+r*cw,pad.t+c*ch,Math.ceil(cw)+.5,Math.ceil(ch)+.5);}}ctx.fillStyle='#aeb9c3';ctx.font='11px system-ui';ctx.fillText('time [s]',w/2-20,h-8);ctx.save();ctx.translate(12,h/2+20);ctx.rotate(-Math.PI/2);ctx.fillText('sector index (0..71)',0,0);ctx.restore();ctx.fillText('red < 1.5 m',pad.l,15);ctx.fillText('yellow < 3 m',pad.l+100,15);ctx.fillText('gray = invalid/unknown',pad.l+200,15);ctx.fillText(`${{o.angle_offset_deg??'?'}} deg offset, ${{o.increment_deg??'?'}} deg increment`,w-210,15);
}}

const SCENE3D = {{azimuth:-0.85,elevation:0.55,distance:1,target:[0,0,0],drag:null,fitPending:true}};

function nedToEnu(x,y,z) {{ return [y,x,-z]; }}

function obstacleVertices(obstacle) {{
  const c=obstacle.center, yaw=obstacle.yaw_rad||0, co=Math.cos(yaw), si=Math.sin(yaw), out=[];
  if (obstacle.type==='box') {{
    const sx=obstacle.size[0]/2, sy=obstacle.size[1]/2, sz=obstacle.size[2]/2;
    for (const z of [-sz,sz]) for (const y of [-sy,sy]) for (const x of [-sx,sx]) out.push([c[0]+co*x-si*y,c[1]+si*x+co*y,c[2]+z]);
  }} else {{
    const count=16, radius=obstacle.radius, half=obstacle.height/2;
    for (const z of [-half,half]) for (let i=0;i<count;i++) {{const a=2*Math.PI*i/count;out.push([c[0]+radius*Math.cos(a),c[1]+radius*Math.sin(a),c[2]+z]);}}
  }}
  return out;
}}

function obstacleFaces(obstacle, vertices) {{
  if (obstacle.type==='box') return [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];
  const count=16, faces=[];
  for (let i=0;i<count;i++) faces.push([i,(i+1)%count,count+(i+1)%count,count+i]);
  faces.push(Array.from({{length:count}},(_,i)=>i).reverse());faces.push(Array.from({{length:count}},(_,i)=>count+i));
  return faces;
}}

function scenePoints() {{
  const points=[];const tr=REPORT.trajectory;
  if (tr) for (let i=0;i<tr.x.length;i++) if (tr.x[i]!==null&&tr.y[i]!==null&&tr.z[i]!==null) points.push(nedToEnu(tr.x[i],tr.y[i],tr.z[i]));
  for (const obstacle of (REPORT.world?.obstacles||[])) points.push(...obstacleVertices(obstacle));
  return points.length?points:[[0,0,0]];
}}

function fitScene3D() {{
  const points=scenePoints(),min=[...points[0]],max=[...points[0]];
  points.slice(1).forEach(point=>{{for(let i=0;i<3;i++){{min[i]=Math.min(min[i],point[i]);max[i]=Math.max(max[i],point[i]);}}}});
  const extent=Math.max(max[0]-min[0],max[1]-min[1],max[2]-min[2],10);SCENE3D.target=[(min[0]+max[0])/2,(min[1]+max[1])/2,(min[2]+max[2])/2];SCENE3D.distance=extent*1.8;SCENE3D.fitPending=false;
}}

function projectScenePoint(point,width,height) {{
  const dx=point[0]-SCENE3D.target[0],dy=point[1]-SCENE3D.target[1],dz=point[2]-SCENE3D.target[2],ca=Math.cos(SCENE3D.azimuth),sa=Math.sin(SCENE3D.azimuth);
  const side=ca*dx-sa*dy, forward=sa*dx+ca*dy, ce=Math.cos(SCENE3D.elevation),se=Math.sin(SCENE3D.elevation),vertical=ce*dz-se*forward,depth=se*dz+ce*forward;
  const scale=Math.min(width,height)*0.72/Math.max(1,SCENE3D.distance),perspective=1/(1+depth/(SCENE3D.distance*2.2));
  return {{x:width/2+side*scale*perspective,y:height/2-vertical*scale*perspective,depth}};
}}

function sceneColor(name) {{
  if (name.startsWith('single')) return '#e76f51'; if (name.startsWith('wide')) return '#f4a261'; if (name.startsWith('l_')) return '#52b788';
  if (name.startsWith('tall_column')) return '#9b5de5'; if (name.startsWith('narrow_gate')) return '#f15bb5'; if (name.startsWith('tall_central')) return '#ff9f1c'; if (name.startsWith('u_')) return '#4d7cfe'; return '#7f8c8d';
}}

function bindScene3D(canvas) {{
  if (canvas.dataset.interactive==='true') return; canvas.dataset.interactive='true';
  canvas.addEventListener('wheel',event=>{{event.preventDefault();SCENE3D.distance*=event.deltaY<0?0.86:1.16;SCENE3D.distance=Math.max(2,Math.min(10000,SCENE3D.distance));drawScene3D();}},{{passive:false}});
  canvas.addEventListener('pointerdown',event=>{{canvas.setPointerCapture(event.pointerId);SCENE3D.drag={{x:event.clientX,y:event.clientY,pan:event.shiftKey||event.button===2}};canvas.classList.add('dragging');}});
  canvas.addEventListener('pointermove',event=>{{if(!SCENE3D.drag)return;const dx=event.clientX-SCENE3D.drag.x,dy=event.clientY-SCENE3D.drag.y;if(SCENE3D.drag.pan){{const scale=SCENE3D.distance/Math.min(canvas.clientWidth,canvas.clientHeight)*2;SCENE3D.target[0]-=dx*scale;SCENE3D.target[1]+=dy*scale;}}else{{SCENE3D.azimuth+=dx*0.008;SCENE3D.elevation=Math.max(-1.35,Math.min(1.35,SCENE3D.elevation+dy*0.008));}}SCENE3D.drag={{x:event.clientX,y:event.clientY,pan:SCENE3D.drag.pan}};drawScene3D();}});
  const stop=event=>{{SCENE3D.drag=null;canvas.classList.remove('dragging');if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);}};canvas.addEventListener('pointerup',stop);canvas.addEventListener('pointercancel',stop);canvas.addEventListener('dblclick',()=>{{SCENE3D.fitPending=true;fitScene3D();drawScene3D();}});canvas.addEventListener('contextmenu',event=>event.preventDefault());
}}

function resetScene3D() {{ SCENE3D.fitPending=true;fitScene3D();drawScene3D(); }}

function drawScene3D() {{
  const canvas=document.getElementById('scene3d');if(!canvas)return;const rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,width=Math.max(760,rect.width),height=Math.max(420,rect.height);canvas.width=width*dpr;canvas.height=height*dpr;const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#0f151a';ctx.fillRect(0,0,width,height);if(SCENE3D.fitPending)fitScene3D();
  const project=point=>projectScenePoint(point,width,height),points=scenePoints();ctx.font='12px system-ui';ctx.lineWidth=1;
  function segment(a,b,color,dash=[]){{const pa=project(a),pb=project(b);ctx.strokeStyle=color;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(pa.x,pa.y);ctx.lineTo(pb.x,pb.y);ctx.stroke();ctx.setLineDash([]);}}
  const extent=Math.max(SCENE3D.distance/1.8,10),step=10,min=-Math.ceil(extent/step)*step,max=Math.ceil(extent/step)*step;
  for(let x=min;x<=max;x+=step)segment([x,min,0],[x,max,0],'#27343d');for(let y=min;y<=max;y+=step)segment([min,y,0],[max,y,0],'#27343d');
  segment([0,0,0],[extent*.35,0,0],'#e76f51');segment([0,0,0],[0,extent*.35,0],'#52b788');segment([0,0,0],[0,0,extent*.35],'#4fc3f7');ctx.fillStyle='#e76f51';ctx.fillText('E / X',project([extent*.35,0,0]).x+5,project([extent*.35,0,0]).y);ctx.fillStyle='#52b788';ctx.fillText('N / Y',project([0,extent*.35,0]).x+5,project([0,extent*.35,0]).y);ctx.fillStyle='#4fc3f7';ctx.fillText('Up / Z',project([0,0,extent*.35]).x+5,project([0,0,extent*.35]).y);
  const renderables=[];for(const obstacle of (REPORT.world?.obstacles||[])){{const vertices=obstacleVertices(obstacle),faces=obstacleFaces(obstacle,vertices);for(const face of faces){{const projected=face.map(index=>project(vertices[index])),depth=projected.reduce((sum,p)=>sum+p.depth,0)/projected.length;renderables.push({{projected,depth,color:sceneColor(obstacle.name)}});}}}}renderables.sort((a,b)=>a.depth-b.depth);for(const face of renderables){{ctx.fillStyle=face.color+'aa';ctx.strokeStyle=face.color;ctx.beginPath();face.projected.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fill();ctx.stroke();}}
  function pathFromNed(data,color,dash=[]){{if(!data)return;const path=[];for(let i=0;i<data.x.length;i++)if(data.x[i]!==null&&data.y[i]!==null&&data.z[i]!==null)path.push(project(nedToEnu(data.x[i],data.y[i],data.z[i])));if(path.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=2.5;ctx.setLineDash(dash);ctx.beginPath();path.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.stroke();ctx.setLineDash([]);}}
  pathFromNed(REPORT.trajectory,'#4fc3f7');if(REPORT.position_setpoint){{const data={{x:REPORT.position_setpoint.v.map(v=>v&&v[0]),y:REPORT.position_setpoint.v.map(v=>v&&v[1]),z:REPORT.position_setpoint.v.map(v=>v&&v[2])}};pathFromNed(data,'#f2cc60',[6,4]);}}
  if(REPORT.trajectory){{const i=REPORT.trajectory.x.length-1;if(REPORT.trajectory.x[i]!==null){{const p=project(nedToEnu(REPORT.trajectory.x[i],REPORT.trajectory.y[i],REPORT.trajectory.z[i]));ctx.fillStyle='#ffffff';ctx.beginPath();ctx.arc(p.x,p.y,5,0,2*Math.PI);ctx.fill();}}}}
  ctx.fillStyle='#aeb9c3';ctx.fillText('drag = rotate | Shift + drag = pan | wheel = zoom | double-click = fit',18,22);ctx.fillStyle='#4fc3f7';ctx.fillRect(18,34,16,3);ctx.fillText('UAV actual',40,39);ctx.fillStyle='#f2cc60';ctx.fillRect(120,34,16,3);ctx.fillText('position setpoint',142,39);ctx.fillStyle='#aeb9c3';ctx.fillText(`${{REPORT.world?.obstacle_count||0}} obstacles from Gazebo SDF`,width-190,22);bindScene3D(canvas);
}}

function installResetButtons() {{
  document.querySelectorAll('canvas[id]').forEach(canvas=>{{if(canvas.id==='scene3d'||canvas.id==='mode'||canvas.id==='heatmap')return;const heading=canvas.parentElement.querySelector('h2');if(!heading||heading.querySelector('button'))return;heading.classList.add('plot-heading');const button=document.createElement('button');button.className='reset-view';button.type='button';button.textContent='Reset view';button.title='Reset zoom and pan';button.addEventListener('click',()=>canvas.id==='trajectory'?resetTrajectory():resetChart(canvas.id));heading.appendChild(button);}});
  const scene=document.getElementById('scene3d'),sceneHeading=scene?.parentElement.querySelector('h2');if(scene&&sceneHeading&&!sceneHeading.querySelector('button')){{sceneHeading.classList.add('plot-heading');const button=document.createElement('button');button.className='reset-view';button.type='button';button.textContent='Reset 3D view';button.title='Reset 3D camera';button.addEventListener('click',resetScene3D);sceneHeading.appendChild(button);}}
}}

function render() {{
  const s=REPORT.summary||{{}},p=REPORT.params||{{}};const cards=[['Duration',`${{Number(s.duration_s||0).toFixed(1)}} s`],['Max actual speed',s.max_actual_speed_m_s===null?'n/a':`${{Number(s.max_actual_speed_m_s).toFixed(2)}} m/s`],['Max AUTO_MISSION speed',s.max_auto_mission_speed_m_s===null?'n/a':`${{Number(s.max_auto_mission_speed_m_s).toFixed(2)}} m/s`],['Min logged clearance',s.min_logged_clearance_m===null?'n/a':`${{Number(s.min_logged_clearance_m).toFixed(2)}} m`],['Sensor gaps',String(s.sensor_gap_events||0)],['Internal POSCTL requests',String(s.internal_posctl_requests||0)]];document.getElementById('summary').innerHTML=cards.map(c=>`<div class="card"><div class="muted">${{c[0]}}</div><strong>${{c[1]}}</strong></div>`).join('');document.getElementById('notes').innerHTML=REPORT.notes.map(n=>`<div>${{n}}</div>`).join('');document.getElementById('events').innerHTML=REPORT.events.length?REPORT.events.map(e=>`<tr><td>${{Number(e.time).toFixed(2)}} s${{e.end_time!==undefined?` -> ${{Number(e.end_time).toFixed(2)}} s`:''}}</td><td class="${{e.severity}}">${{e.severity}}</td><td>${{e.kind}}</td><td>${{e.message}}</td><td>${{Number(e.duration_s||0).toFixed(2)}} s</td></tr>`).join(''):'<tr><td colspan="5">No event detected.</td></tr>';document.getElementById('topics').innerHTML=Object.entries(REPORT.meta.topics).map(([n,v])=>`<tr><td><code>${{n}}</code></td><td class="${{v?'info':'critical'}}">${{v?'yes':'missing'}}</td></tr>`).join('');document.getElementById('params').innerHTML=Object.entries(p).map(([n,v])=>`<tr><td><code>${{n}}</code></td><td>${{v}}</td><td>${{(REPORT.parameter_changes[n]||[]).map(c=>`${{Number(c.time).toFixed(2)}}s -> ${{c.value}}`).join('<br>')||'none'}}</td></tr>`).join('');const world=REPORT.world||{{}},worldInfo=document.getElementById('world-info');if(worldInfo)worldInfo.textContent=world.error?`World: ${{world.error}} (${{world.path}})`: `World: ${{world.path}} | ${{world.obstacle_count||0}} obstacles | ${{world.px4_transform}}`;
  installResetButtons();drawTrajectory();drawMode();drawHeatmap();drawScene3D();const tr=REPORT.trajectory,sp=REPORT.velocity_setpoint,manual=REPORT.manual_control,acc=REPORT.vehicle_acceleration_frd,cc=REPORT.collision_constraints,o=REPORT.obstacle,a=REPORT.attitude;drawChart('velocity',[...(tr?[{{name:'actual vx',color:'#4fc3f7',t:tr.t,v:tr.vx}},{{name:'actual vy',color:'#9cdcfe',t:tr.t,v:tr.vy}},{{name:'actual speed',color:'#f2cc60',t:tr.t,v:tr.speed}}]:[]),...(sp?[{{name:'setpoint vx',color:'#ff9f43',t:sp.t,v:sp.v.map(v=>v&&v[0])}},{{name:'setpoint vy',color:'#ff7b72',t:sp.t,v:sp.v.map(v=>v&&v[1])}}]:[])],'m/s',[...(p.CP_VFH_MAX_VEL!==undefined?[{{value:Number(p.CP_VFH_MAX_VEL),color:'#f2cc60',label:'VFH cap'}}]:[])]);drawChart('joystick',manual?[{{name:'roll',color:'#ff7b72',t:manual.t,v:manual.v.map(v=>v&&v[0])}},{{name:'pitch',color:'#4fc3f7',t:manual.t,v:manual.v.map(v=>v&&v[1])}},{{name:'yaw',color:'#f2cc60',t:manual.t,v:manual.v.map(v=>v&&v[2])}},{{name:'throttle',color:'#9cdcfe',t:manual.t,v:manual.v.map(v=>v&&v[3])}}]:[],'normalized');drawChart('constraints',cc&&cc.original&&cc.adapted?[{{name:'original X',color:'#aeb9c3',t:cc.original.t,v:cc.original.v.map(v=>v&&v[0])}},{{name:'adapted X',color:'#ff7b72',t:cc.adapted.t,v:cc.adapted.v.map(v=>v&&v[0])}},{{name:'original Y',color:'#9cdcfe',t:cc.original.t,v:cc.original.v.map(v=>v&&v[1])}},{{name:'adapted Y',color:'#f2cc60',t:cc.adapted.t,v:cc.adapted.v.map(v=>v&&v[1])}}]:[],'logged XY setpoint (manual path: m/s2)');drawChart('acceleration',acc?[{{name:'body X forward',color:'#4fc3f7',t:acc.t,v:acc.v.map(v=>v&&v[0])}},{{name:'body Y right',color:'#f2cc60',t:acc.t,v:acc.v.map(v=>v&&v[1])}},{{name:'body Z down',color:'#ff7b72',t:acc.t,v:acc.v.map(v=>v&&v[2])}}]:[],'m/s2');drawChart('clearance',o?[{{name:'nearest valid sector',color:'#ff7b72',t:o.t,v:o.clearance}},...(o.max_distance?[{{name:'sensor max range',color:'#aeb9c3',t:o.t,v:o.max_distance}}]:[])]:[],'m',[...(p.CP_DIST>0?[{{value:Number(p.CP_DIST),color:'#f2cc60',label:'final CP_DIST'}}]:[]),...(p.CP_AUTO_DIST>0?[{{value:Number(p.CP_AUTO_DIST),color:'#4fc3f7',label:'final CP_AUTO_DIST'}}]:[]),...(p.CP_VFH_STOP_DIST>0?[{{value:Number(p.CP_VFH_STOP_DIST),color:'#ff7b72',label:'VFH stop'}}]:[])]);drawChart('validity',o?[{{name:'valid sectors / 72',color:'#4fc3f7',t:o.t,v:o.valid_fraction.map(v=>v===null?null:v*72)}}]:[],'count',[{{value:72,color:'#aeb9c3',label:'72 sectors'}}]);drawChart('attitude',a?[{{name:'roll',color:'#ff7b72',t:a.t,v:a.roll_deg}},{{name:'pitch',color:'#f2cc60',t:a.t,v:a.pitch_deg}},{{name:'yaw',color:'#4fc3f7',t:a.t,v:a.yaw_deg}}]:[],'deg');
  drawChart('sensor_status',o&&o.sensor_status?[{{name:'obstacle data status',color:'#ff7b72',t:o.sensor_status.t,v:o.sensor_status.v}}]:[],'status',[{{value:1,color:'#4fc3f7',label:'valid'}},{{value:0,color:'#ff7b72',label:'lost / invalid'}}]);
}}
window.addEventListener('resize',render);render();
</script>
</body>
</html>
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path, help="PX4 .ulg file")
    parser.add_argument("-o", "--output", type=Path, help="output HTML path")
    parser.add_argument("--world", type=Path,
                        help="Gazebo SDF/world file; defaults to vfh_obstacle_course.sdf")
    parser.add_argument("--title", default="PX4 Collision Prevention flight report")
    parser.add_argument("--plot-scale", type=float, default=3.0,
                        help="horizontal report width multiplier (default: 3)")
    args = parser.parse_args()

    output = args.output or args.ulog.with_suffix(".cp-report.html")
    report = build_report(args.ulog, args.world)
    output.write_text(html_report(report, args.title, args.plot_scale), encoding="utf-8")
    print(f"Wrote {output}")
    print(f"Duration: {report['meta']['duration_s']:.1f} s")
    print(f"Topics present: {sum(report['meta']['topics'].values())}/{len(report['meta']['topics'])}")
    summary = report["summary"]
    print(f"Min clearance: {summary['min_logged_clearance_m']}")
    print(f"Sensor gaps: {summary['sensor_gap_events']}")
    print(f"Internal POSCTL requests: {summary['internal_posctl_requests']}")
    print(f"Events: {len(report['events'])}")


if __name__ == "__main__":
    main()
