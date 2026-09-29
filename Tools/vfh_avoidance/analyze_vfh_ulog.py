#!/usr/bin/env python3
"""Create a self-contained HTML report for a PX4 VFH avoidance flight.

The script intentionally uses only pyulog, numpy and the Python standard
library. It works with the topics already logged by PX4 and does not require
an internet connection or a JavaScript plotting package.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from pyulog import ULog


MAX_POINTS = 6000
PARAMETER_NAMES = (
    "CP_AUTO_EN",
    "CP_AUTO_DIST",
    "CP_AUTO_MODE",
    "CP_VFH_MAX_VEL",
    "CP_DIST",
    "CP_DELAY",
    "CP_MAX_ANGLE",
    "CP_BRAKE_ANGLE",
    "MPC_XY_CRUISE",
    "MPC_XY_VEL_MAX",
    "MPC_JERK_AUTO",
)


def dataset_for(ulog: ULog, topic: str) -> Any | None:
    """Return the first instance of a logged topic, if present."""
    for dataset in ulog.data_list:
        if dataset.name == topic:
            return dataset

    return None


def field_for(dataset: Any | None, field: str) -> np.ndarray | None:
    if dataset is None or field not in dataset.data:
        return None

    return np.asarray(dataset.data[field])


def common_timestamp(dataset: Any | None, *fields: np.ndarray | None) -> np.ndarray | None:
    timestamp = field_for(dataset, "timestamp")

    if timestamp is None or any(values is None for values in fields):
        return None

    length = min(len(timestamp), *(len(values) for values in fields if values is not None))
    return timestamp[:length].astype(np.float64)


def finite_value(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None

    return number if math.isfinite(number) else None


def downsample(timestamp: np.ndarray, values: list[float | None], limit: int = MAX_POINTS) -> tuple[list[float], list[float | None]]:
    if len(timestamp) <= limit:
        return timestamp.tolist(), values

    indices = np.linspace(0, len(timestamp) - 1, limit, dtype=np.int64)
    return timestamp[indices].tolist(), [values[index] for index in indices]


def series(dataset: Any | None, field: str, start_us: float, scale: float = 1.0) -> dict[str, Any] | None:
    values = field_for(dataset, field)
    timestamp = common_timestamp(dataset, values)

    if timestamp is None:
        return None

    values = values[:len(timestamp)].astype(np.float64) * scale
    time_s = (timestamp - start_us) * 1e-6
    time_list, value_list = downsample(time_s, [finite_value(value) for value in values])
    return {"t": time_list, "v": value_list}


def vector_series(dataset: Any | None, fields: tuple[str, ...], start_us: float, scales: tuple[float, ...] | None = None) -> dict[str, Any] | None:
    values = tuple(field_for(dataset, field) for field in fields)
    timestamp = common_timestamp(dataset, *values)

    if timestamp is None:
        return None

    length = len(timestamp)
    arrays = tuple(value[:length].astype(np.float64) for value in values)
    if scales is not None:
        arrays = tuple(value * scale for value, scale in zip(arrays, scales))

    time_s = (timestamp - start_us) * 1e-6
    indices = np.linspace(0, length - 1, min(length, MAX_POINTS), dtype=np.int64)
    return {
        "t": time_s[indices].tolist(),
        "v": [[finite_value(array[index]) for array in arrays] for index in indices],
    }


def obstacle_clearance_series(dataset: Any | None, start_us: float) -> dict[str, Any] | None:
    """Return the nearest valid obstacle sector, not obstacle_distance.min_distance.

    The message's min_distance field is the sensor minimum operating range. The
    actual clearance is the minimum of distances[0..71] after invalid and
    out-of-range returns have been removed.
    """
    timestamp = field_for(dataset, "timestamp")
    distance_fields = tuple(field_for(dataset, f"distances[{index}]") for index in range(72))
    max_distance = field_for(dataset, "max_distance")

    if timestamp is None or any(values is None for values in distance_fields):
        return None

    length = min(len(timestamp), *(len(values) for values in distance_fields))
    distances = np.column_stack([values[:length].astype(np.float64) for values in distance_fields])
    timestamp = timestamp[:length].astype(np.float64)
    valid = np.isfinite(distances) & (distances < 65535.0)

    if max_distance is not None:
        sensor_max = max_distance[:length].astype(np.float64)
        valid &= distances <= sensor_max[:, None]

    row_has_value = valid.any(axis=1)
    nearest_cm = np.full(length, np.nan)
    nearest_cm[row_has_value] = np.min(np.where(valid[row_has_value],
                                                distances[row_has_value], np.inf), axis=1)
    time_s = (timestamp - start_us) * 1e-6
    values = [finite_value(value * 0.01) for value in nearest_cm]
    time_list, value_list = downsample(time_s, values)
    return {"t": time_list, "v": value_list}


def parameter_value(ulog: ULog, name: str, default: Any = None) -> Any:
    changed = getattr(ulog, "changed_parameters", {})
    initial = getattr(ulog, "initial_parameters", {})

    if name in changed:
        value = changed[name]
        if isinstance(value, (list, tuple)):
            return value[-1]
        return value

    return initial.get(name, default)


def collect_start_time(datasets: list[Any | None]) -> float:
    timestamps = []

    for dataset in datasets:
        timestamp = field_for(dataset, "timestamp")
        if timestamp is not None and len(timestamp):
            timestamps.append(float(timestamp[0]))

    return min(timestamps) if timestamps else 0.0


def paired_trajectory(position: Any | None, start_us: float) -> dict[str, Any] | None:
    values = tuple(field_for(position, field) for field in ("x", "y", "z", "vx", "vy", "vz"))
    timestamp = common_timestamp(position, *values)

    if timestamp is None:
        return None

    length = len(timestamp)
    arrays = tuple(value[:length].astype(np.float64) for value in values)
    speed = np.sqrt(arrays[3] ** 2 + arrays[4] ** 2)
    time_s = (timestamp - start_us) * 1e-6
    indices = np.linspace(0, length - 1, min(length, MAX_POINTS), dtype=np.int64)

    return {
        "t": time_s[indices].tolist(),
        "x": [finite_value(arrays[0][index]) for index in indices],
        "y": [finite_value(arrays[1][index]) for index in indices],
        "z": [finite_value(arrays[2][index]) for index in indices],
        "vx": [finite_value(arrays[3][index]) for index in indices],
        "vy": [finite_value(arrays[4][index]) for index in indices],
        "vz": [finite_value(arrays[5][index]) for index in indices],
        "speed": [finite_value(speed[index]) for index in indices],
    }


def attitude_yaw(attitude: Any | None, start_us: float) -> dict[str, Any] | None:
    quaternion = tuple(field_for(attitude, f"q[{index}]") for index in range(4))
    timestamp = common_timestamp(attitude, *quaternion)

    if timestamp is None:
        return None

    length = len(timestamp)
    q = tuple(values[:length].astype(np.float64) for values in quaternion)
    yaw = np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                     1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
    roll = np.arctan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                      1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))
    pitch = np.arcsin(np.clip(2.0 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0))
    indices = np.linspace(0, length - 1, min(length, MAX_POINTS), dtype=np.int64)
    time_s = (timestamp - start_us) * 1e-6

    return {
        "t": time_s[indices].tolist(),
        "yaw_deg": [finite_value(math.degrees(yaw[index])) for index in indices],
        "roll_deg": [finite_value(math.degrees(roll[index])) for index in indices],
        "pitch_deg": [finite_value(math.degrees(pitch[index])) for index in indices],
    }


def add_threshold_events(events: list[dict[str, Any]], data: dict[str, Any] | None,
                         value_key: str, threshold: float, message: str,
                         direction: str = "below") -> None:
    if data is None:
        return

    active = []
    for timestamp, value in zip(data["t"], data[value_key]):
        if value is None:
            active.append(False)
        elif direction == "above":
            active.append(value > threshold)
        else:
            active.append(value < threshold)

    start = None
    for index, is_active in enumerate(active + [False]):
        if is_active and start is None:
            start = index
        elif not is_active and start is not None:
            end = index - 1
            if end >= start:
                events.append({
                    "time": data["t"][start],
                    "severity": "warning",
                    "message": f"{message} ({data[value_key][start]:.2f} at start)",
                    "duration_s": max(0.0, data["t"][end] - data["t"][start]),
                })
            start = None


def add_gap_events(events: list[dict[str, Any]], data: dict[str, Any] | None,
                   timeout_s: float, message: str) -> None:
    if data is None or len(data["t"]) < 2:
        return

    for first, second in zip(data["t"], data["t"][1:]):
        if second - first > timeout_s:
            events.append({
                "time": first,
                "severity": "critical",
                "message": message,
                "duration_s": second - first,
            })


def build_report(path: Path) -> dict[str, Any]:
    ulog = ULog(str(path))
    datasets = [dataset_for(ulog, topic) for topic in (
        "vehicle_local_position",
        "vehicle_local_position_setpoint",
        "vehicle_attitude",
        "obstacle_distance_fused",
        "obstacle_distance",
        "collision_constraints",
        "vehicle_status",
        "position_setpoint_triplet",
    )]
    start_us = float(getattr(ulog, "start_timestamp", 0) or collect_start_time(datasets))

    position = datasets[0]
    position_setpoint = datasets[1]
    attitude = datasets[2]
    # The fused topic can contain zero-timestamp placeholder samples while the
    # raw bridge topic remains monotonic. Prefer raw data for timing/dropout
    # analysis and keep the fused topic available to PX4's control path.
    obstacle = datasets[4] or datasets[3]
    constraints = datasets[5]

    trajectory = paired_trajectory(position, start_us)
    actual_speed = None
    if trajectory is not None:
        actual_speed = {"t": trajectory["t"], "v": trajectory["speed"]}

    obstacle_distance = obstacle_clearance_series(obstacle, start_us)
    obstacle_max_distance = series(obstacle, "max_distance", start_us, 0.01)
    position_sp = vector_series(position_setpoint, ("x", "y", "z"), start_us)
    velocity_sp = vector_series(position_setpoint, ("vx", "vy", "vz"), start_us)
    original_accel = vector_series(constraints, ("original_setpoint[0]", "original_setpoint[1]"), start_us)
    adapted_accel = vector_series(constraints, ("adapted_setpoint[0]", "adapted_setpoint[1]"), start_us)
    yaw = attitude_yaw(attitude, start_us)

    params = {name: parameter_value(ulog, name) for name in PARAMETER_NAMES}
    params = {name: finite_value(value) if isinstance(value, (int, float, np.number)) else value
              for name, value in params.items() if value is not None}
    vfh_max_velocity = float(params.get("CP_VFH_MAX_VEL", 3.0))
    cp_dist = float(params.get("CP_DIST", -1.0))
    events: list[dict[str, Any]] = []

    add_threshold_events(events, actual_speed, "v", vfh_max_velocity * 1.05,
                         f"Actual horizontal speed exceeded CP_VFH_MAX_VEL={vfh_max_velocity:.2f} m/s",
                         "above")
    if cp_dist > 0:
        add_threshold_events(events, obstacle_distance, "v", cp_dist,
                             f"Obstacle clearance below CP_DIST={cp_dist:.2f} m")
    add_gap_events(events, obstacle_distance, 0.7,
                   "Obstacle distance stream gap exceeded 0.7 s")
    add_threshold_events(events, yaw, "roll_deg", float(params.get("CP_MAX_ANGLE", 10.0)) * 1.25,
                         "Roll exceeded 125% of CP_MAX_ANGLE", "above") if yaw else None
    add_threshold_events(events, yaw, "pitch_deg", float(params.get("CP_MAX_ANGLE", 10.0)) * 1.25,
                         "Pitch exceeded 125% of CP_MAX_ANGLE", "above") if yaw else None

    events.sort(key=lambda event: event["time"])
    duration = 0.0
    for data in (trajectory, obstacle_distance, yaw):
        if data and data.get("t"):
            duration = max(duration, float(data["t"][-1]))

    return {
        "meta": {
            "ulog": str(path),
            "duration_s": duration,
            "topics": {topic: dataset_for(ulog, topic) is not None for topic in (
                "vehicle_local_position",
                "vehicle_local_position_setpoint",
                "vehicle_attitude",
                "obstacle_distance_fused",
                "obstacle_distance",
                "collision_constraints",
                "vehicle_status",
                "position_setpoint_triplet",
            )},
        },
        "params": params,
        "trajectory": trajectory,
        "actual_speed": actual_speed,
        "position_setpoint": position_sp,
        "velocity_setpoint": velocity_sp,
        "obstacle_distance": obstacle_distance,
        "obstacle_max_distance": obstacle_max_distance,
        "original_accel": original_accel,
        "adapted_accel": adapted_accel,
        "attitude": yaw,
        "events": events,
        "notes": [
            "VFH selected_direction, cost and free_sector_count are not structured ULog signals yet.",
            "They currently appear in the rate-limited PX4_INFO message 'VFH control'.",
            "A future uORB debug topic should publish those fields for exact planner-level plots.",
        ],
    }


def html_report(report: dict[str, Any], title: str) -> str:
    payload = json.dumps(report, allow_nan=False, separators=(",", ":"))
    safe_title = html.escape(title)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{safe_title}</title>
<style>
  :root {{ color-scheme: dark; font-family: system-ui, sans-serif; }}
  body {{ margin: 0; background: #101418; color: #e8edf2; }}
  main {{ max-width: 1320px; margin: auto; padding: 24px; }}
  h1 {{ margin-top: 0; font-size: 1.55rem; }}
  h2 {{ font-size: 1.05rem; margin: 0 0 10px; }}
  .summary {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; margin: 14px 0; }}
  .card {{ background: #192027; border: 1px solid #303b45; border-radius: 6px; padding: 12px; }}
  .muted {{ color: #aeb9c3; font-size: .9rem; }}
  .plots {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(480px, 1fr)); gap: 12px; }}
  canvas {{ width: 100%; height: 300px; background: #12181d; border-radius: 4px; }}
  #trajectory {{ height: 420px; }}
  table {{ width: 100%; border-collapse: collapse; font-size: .9rem; }}
  th, td {{ text-align: left; padding: 7px; border-bottom: 1px solid #303b45; }}
  .critical {{ color: #ff7b72; }} .warning {{ color: #f2cc60; }}
  code {{ color: #9cdcfe; }}
</style>
</head>
<body>
<main>
  <h1>{safe_title}</h1>
  <div id="summary" class="summary"></div>
  <section class="card">
    <h2>How to read this report</h2>
    <p class="muted">Look first for red event markers, speed above the VFH cap, clearance below CP_DIST, gaps in obstacle data, and a large separation between actual position and the mission setpoint. A smooth return toward the current mission line after the obstacle clears is expected; Navigator still owns the waypoint sequence.</p>
    <p id="notes" class="muted"></p>
  </section>
  <section class="card" style="margin-top:12px">
    <h2>Local trajectory (NED X/Y)</h2>
    <canvas id="trajectory"></canvas>
  </section>
  <section class="plots" style="margin-top:12px">
    <div class="card"><h2>Horizontal speed</h2><canvas id="speed"></canvas></div>
    <div class="card"><h2>Obstacle clearance</h2><canvas id="clearance"></canvas></div>
    <div class="card"><h2>Position setpoint and velocity setpoint</h2><canvas id="setpoints"></canvas></div>
    <div class="card"><h2>Collision Prevention acceleration constraints</h2><canvas id="constraints"></canvas></div>
    <div class="card"><h2>Attitude</h2><canvas id="attitude"></canvas></div>
  </section>
  <section class="card" style="margin-top:12px">
    <h2>Detected events</h2>
    <table><thead><tr><th>Time</th><th>Level</th><th>Message</th><th>Duration</th></tr></thead><tbody id="events"></tbody></table>
  </section>
</main>
<script>
const REPORT = {payload};

function finitePairs(t, v) {{
  const pairs = [];
  if (!t || !v) return pairs;
  for (let i = 0; i < Math.min(t.length, v.length); i++) {{
    if (v[i] !== null && Number.isFinite(v[i])) pairs.push([t[i], v[i]]);
  }}
  return pairs;
}}

function drawChart(canvasId, series, yLabel, horizontalLines=[]) {{
  const canvas = document.getElementById(canvasId);
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(640, rect.width * dpr);
  canvas.height = Math.max(280, rect.height * dpr);
  const ctx = canvas.getContext('2d'); ctx.scale(dpr, dpr);
  const width = rect.width, height = rect.height;
  const pad = {{left: 58, right: 18, top: 22, bottom: 34}};
  const all = series.flatMap(s => finitePairs(s.t, s.v));
  if (!all.length) {{ ctx.fillStyle='#aeb9c3'; ctx.fillText('Topic or field not available in this ULog', 20, 30); return; }}
  const xmin = Math.min(...all.map(p => p[0])), xmax = Math.max(...all.map(p => p[0]));
  let ymin = Math.min(...all.map(p => p[1])), ymax = Math.max(...all.map(p => p[1]));
  horizontalLines.forEach(line => {{ ymin = Math.min(ymin, line.value); ymax = Math.max(ymax, line.value); }});
  if (Math.abs(ymax-ymin) < 1e-6) {{ ymin -= 1; ymax += 1; }}
  const x = t => pad.left + (t-xmin) / Math.max(1e-6, xmax-xmin) * (width-pad.left-pad.right);
  const y = value => height-pad.bottom - (value-ymin) / (ymax-ymin) * (height-pad.top-pad.bottom);
  ctx.fillStyle='#12181d'; ctx.fillRect(0,0,width,height);
  ctx.strokeStyle='#303b45'; ctx.lineWidth=1; ctx.fillStyle='#aeb9c3'; ctx.font='11px system-ui';
  for (let i=0;i<=5;i++) {{ const yy=pad.top+i*(height-pad.top-pad.bottom)/5; ctx.beginPath(); ctx.moveTo(pad.left,yy); ctx.lineTo(width-pad.right,yy); ctx.stroke(); ctx.fillText((ymax-(ymax-ymin)*i/5).toFixed(2), 5, yy+4); }}
  ctx.fillText('time [s]', width/2-20, height-8); ctx.save(); ctx.translate(12,height/2+20); ctx.rotate(-Math.PI/2); ctx.fillText(yLabel,0,0); ctx.restore();
  horizontalLines.forEach(line => {{ ctx.strokeStyle=line.color||'#ff7b72'; ctx.setLineDash([5,4]); ctx.beginPath(); ctx.moveTo(pad.left,y(line.value)); ctx.lineTo(width-pad.right,y(line.value)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle=line.color||'#ff7b72'; ctx.fillText(line.label||line.value, pad.left+5, y(line.value)-4); }});
  series.forEach(s => {{ const pairs=finitePairs(s.t,s.v); if(!pairs.length)return; ctx.strokeStyle=s.color; ctx.lineWidth=1.8; ctx.beginPath(); pairs.forEach((p,i)=>{{if(i===0)ctx.moveTo(x(p[0]),y(p[1]));else ctx.lineTo(x(p[0]),y(p[1]));}}); ctx.stroke(); }});
  let legendX=pad.left; series.forEach(s=>{{ctx.fillStyle=s.color;ctx.fillRect(legendX,6,12,3);ctx.fillText(s.name,legendX+16,10);legendX+=Math.max(90,s.name.length*7+28);}});
}}

function drawTrajectory() {{
  const canvas=document.getElementById('trajectory'), rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  canvas.width=Math.max(640,rect.width*dpr);canvas.height=Math.max(380,rect.height*dpr);const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
  const w=rect.width,h=rect.height,p={{l:58,r:18,t:22,b:42}};const tr=REPORT.trajectory,sp=REPORT.position_setpoint;
  if(!tr){{ctx.fillStyle='#aeb9c3';ctx.fillText('vehicle_local_position not available',20,30);return;}}
  const points=tr.x.map((v,i)=>v!==null&&tr.y[i]!==null?[v,tr.y[i]]:null).filter(Boolean);const targets=sp?sp.v.map(v=>v&&v[0]!==null&&v[1]!==null?[v[0],v[1]]:null).filter(Boolean):[];
  const all=points.concat(targets);let xmin=Math.min(...all.map(p=>p[0])),xmax=Math.max(...all.map(p=>p[0])),ymin=Math.min(...all.map(p=>p[1])),ymax=Math.max(...all.map(p=>p[1]));if(xmax-xmin<1){{xmin-=1;xmax+=1;}}if(ymax-ymin<1){{ymin-=1;ymax+=1;}}const sx=(x)=>p.l+(x-xmin)/(xmax-xmin)*(w-p.l-p.r),sy=(y)=>h-p.b-(y-ymin)/(ymax-ymin)*(h-p.t-p.b);
  ctx.fillStyle='#12181d';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#303b45';ctx.lineWidth=1;for(let i=0;i<=5;i++){{const xx=p.l+i*(w-p.l-p.r)/5,yy=p.t+i*(h-p.t-p.b)/5;ctx.beginPath();ctx.moveTo(xx,p.t);ctx.lineTo(xx,h-p.b);ctx.moveTo(p.l,yy);ctx.lineTo(w-p.r,yy);ctx.stroke();}}
  function line(data,color,dash){{ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dash||[]);ctx.beginPath();data.forEach((q,i)=>{{if(i===0)ctx.moveTo(sx(q[0]),sy(q[1]));else ctx.lineTo(sx(q[0]),sy(q[1]));}});ctx.stroke();ctx.setLineDash([]);}}
  line(points,'#4fc3f7');if(targets.length)line(targets,'#f2cc60',[6,4]);ctx.fillStyle='#aeb9c3';ctx.fillText('X [m]',w/2-15,h-12);ctx.save();ctx.translate(14,h/2+15);ctx.rotate(-Math.PI/2);ctx.fillText('Y [m]',0,0);ctx.restore();ctx.fillStyle='#4fc3f7';ctx.fillRect(p.l,8,14,3);ctx.fillText('actual',p.l+18,12);ctx.fillStyle='#f2cc60';ctx.fillRect(p.l+85,8,14,3);ctx.fillText('mission setpoint',p.l+104,12);
}}

function addSummary() {{
  const p=REPORT.params, m=REPORT.meta;const items=[['Duration',`${{m.duration_s.toFixed(1)}} s`],['VFH cap',`${{Number(p.CP_VFH_MAX_VEL??3).toFixed(2)}} m/s`],['CP_DIST',p.CP_DIST??'not logged'],['CP_AUTO_MODE',p.CP_AUTO_MODE??'not logged'],['Events',REPORT.events.length]];
  document.getElementById('summary').innerHTML=items.map(i=>`<div class="card"><div class="muted">${{i[0]}}</div><strong>${{i[1]}}</strong></div>`).join('');
  document.getElementById('notes').innerHTML=REPORT.notes.map(n=>`<div>${{n}}</div>`).join('');
  document.getElementById('events').innerHTML=REPORT.events.length?REPORT.events.map(e=>`<tr><td>${{e.time.toFixed(2)}} s</td><td class="${{e.severity}}">${{e.severity}}</td><td>${{e.message}}</td><td>${{(e.duration_s||0).toFixed(2)}} s</td></tr>`).join(''):'<tr><td colspan="4">No automatic threshold event detected.</td></tr>';
}}

function render() {{
  addSummary();
  const speed=REPORT.actual_speed?{{t:REPORT.actual_speed.t,v:REPORT.actual_speed.v}}:null;const cap=Number(REPORT.params.CP_VFH_MAX_VEL??3);
  drawTrajectory();drawChart('speed',[...(speed?[{{name:'actual',color:'#4fc3f7',...speed}}]:[])], 'm/s',[{{value:cap,color:'#f2cc60',label:'VFH cap'}}]);
  drawChart('clearance',[...(REPORT.obstacle_distance?[{{name:'min distance',color:'#ff7b72',...REPORT.obstacle_distance}}]:[]),...(REPORT.obstacle_max_distance?[{{name:'max range',color:'#aeb9c3',...REPORT.obstacle_max_distance}}]:[])], 'm',[...(Number(REPORT.params.CP_DIST)>0?[{{value:Number(REPORT.params.CP_DIST),color:'#f2cc60',label:'CP_DIST'}}]:[])]);
  const sp=REPORT.position_setpoint?REPORT.position_setpoint.v.map(v=>v&&v[0]!==null&&v[1]!==null?Math.hypot(v[0],v[1]):null):null;const vp=REPORT.velocity_setpoint?REPORT.velocity_setpoint.v.map(v=>v&&v[0]!==null&&v[1]!==null?Math.hypot(v[0],v[1]):null):null;const st=REPORT.position_setpoint?.t||REPORT.velocity_setpoint?.t||[];drawChart('setpoints',[...(sp?[{{name:'position setpoint norm',color:'#f2cc60',t:REPORT.position_setpoint.t,v:sp}}]:[]),...(vp?[{{name:'velocity setpoint',color:'#4fc3f7',t:REPORT.velocity_setpoint.t,v:vp}}]:[])],'m or m/s');
  const oa=REPORT.original_accel?REPORT.original_accel.v.map(v=>v&&v[0]!==null&&v[1]!==null?Math.hypot(v[0],v[1]):null):null;const aa=REPORT.adapted_accel?REPORT.adapted_accel.v.map(v=>v&&v[0]!==null&&v[1]!==null?Math.hypot(v[0],v[1]):null):null;drawChart('constraints',[...(oa?[{{name:'original XY',color:'#aeb9c3',t:REPORT.original_accel.t,v:oa}}]:[]),...(aa?[{{name:'adapted XY',color:'#ff7b72',t:REPORT.adapted_accel.t,v:aa}}]:[])],'m/s2');
  drawChart('attitude',[...(REPORT.attitude?[{{name:'roll',color:'#ff7b72',t:REPORT.attitude.t,v:REPORT.attitude.roll_deg}},{{name:'pitch',color:'#f2cc60',t:REPORT.attitude.t,v:REPORT.attitude.pitch_deg}},{{name:'yaw',color:'#4fc3f7',t:REPORT.attitude.t,v:REPORT.attitude.yaw_deg}}]:[])],'deg');
}}
window.addEventListener('resize', render); render();
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path, help="PX4 .ulg file")
    parser.add_argument("-o", "--output", type=Path, help="output HTML path")
    parser.add_argument("--title", default="PX4 VFH flight report")
    args = parser.parse_args()

    output = args.output or args.ulog.with_suffix(".vfh.html")
    report = build_report(args.ulog)
    output.write_text(html_report(report, args.title), encoding="utf-8")
    print(f"Wrote {output}")
    print(f"Duration: {report['meta']['duration_s']:.1f} s")
    print(f"Detected events: {len(report['events'])}")
    missing = [name for name, present in report["meta"]["topics"].items() if not present]
    if missing:
        print("Missing topics: " + ", ".join(missing))


if __name__ == "__main__":
    main()
