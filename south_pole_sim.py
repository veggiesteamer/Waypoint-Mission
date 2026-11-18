"""South Pole baseline mission: time, energy and data feasibility check.

Reads the ConOps input workbook (README / Parameters / Modes / Procedures /
Mission / Activity map / Phases sheets), expands the Activity map into a
sequential mode-by-mode timeline and checks it against the lunar-day window.

    python south_pole_sim.py [path/to/Input_Conops_vN.xlsx]

No map, no path planning: each phase's walked distance (Phases sheet) is
split evenly over that phase's walking steps (steps whose duration override is
"from phase"). Spreadsheet formulas are evaluated here, so the workbook does
not need to have been recalculated/saved by Excel.

Outputs go to experiments/south_pole/run_<timestamp>/: timeline.csv,
summary.txt, stats.png, timeline_phases.png, data_overview.png.
"""
import csv
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

import openpyxl
from openpyxl.utils import column_index_from_string

DEFAULT_WORKBOOK = Path(__file__).parent / "user_data" / "Input_Conops_v12.xlsx"
OUT_ROOT = Path(__file__).parent / "experiments" / "south_pole"

# Reference categorical palette, slots 1-5 in fixed order (one per phase).
PHASE_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]


# --------------------------------------------------------------------------
# Workbook access with a small formula evaluator
# --------------------------------------------------------------------------

class Blank(float):
    """An empty cell: 0 in arithmetic, equal to "" in comparisons (as in Excel)."""
    def __new__(cls):
        return super().__new__(cls, 0.0)

    def __eq__(self, other):
        if isinstance(other, str):
            return other == ""
        return other is None or float(other) == 0.0

    __hash__ = float.__hash__


_RANGE = re.compile(r"(?:'?([A-Za-z][\w ]*)'?!)?\$?([A-Z]{1,3})\$?(\d+):\$?([A-Z]{1,3})\$?(\d+)")
_REF = re.compile(r"(?:'?([A-Za-z][\w ]*)'?!)?\$?\b([A-Z]{1,3})\$?(\d+)\b")


_NAME = re.compile(r"\b([a-z_][a-z0-9_]*)\b(?!\s*\()")


class Workbook:
    def __init__(self, path):
        self.wb = openpyxl.load_workbook(path)  # formulas, not cached values
        self._cache = {}
        # defined names (one per parameter key) -> (sheet, row, col)
        self.names = {}
        for name, dn in self.wb.defined_names.items():
            for sheet, coord in dn.destinations:
                cell = self.wb[sheet][coord.replace("$", "")]
                self.names[name.lower()] = (sheet, cell.row, cell.column)

    def formulas(self):
        """Every formula in the workbook, for usage checks."""
        for ws in self.wb:
            for row in ws.iter_rows():
                for c in row:
                    if isinstance(c.value, str) and c.value.startswith("="):
                        yield ws.title, c.coordinate, c.value

    def _names_in(self, expr):
        parts = expr.split('"')  # skip quoted strings
        return {m.group(1) for i, p in enumerate(parts) if i % 2 == 0 for m in _NAME.finditer(p)
                if m.group(1) in self.names}

    def value(self, sheet, row, col):
        key = (sheet, row, col)
        if key not in self._cache:
            raw = self.wb[sheet].cell(row, col).value
            if isinstance(raw, str) and raw.startswith("="):
                self._cache[key] = self._eval(sheet, raw[1:], f"{sheet}!{self.wb[sheet].cell(row, col).coordinate}")
            else:
                self._cache[key] = Blank() if raw is None or raw == "" else raw
        return self._cache[key]

    def _eval(self, sheet, expr, where):
        def rng(m):
            sh = m.group(1) or sheet
            c0, r0 = column_index_from_string(m.group(2)), int(m.group(3))
            c1, r1 = column_index_from_string(m.group(4)), int(m.group(5))
            return "[" + ",".join(f"_v({sh!r},{r},{c})" for r in range(r0, r1 + 1) for c in range(c0, c1 + 1)) + "]"

        def ref(m):
            sh = m.group(1) or sheet
            return f"_v({sh!r},{int(m.group(3))},{column_index_from_string(m.group(2))})"

        def name(m):
            if m.group(1) not in self.names:
                return m.group(0)
            sh, r, c = self.names[m.group(1)]
            return f"_v({sh!r},{r},{c})"

        py = _RANGE.sub(rng, expr)
        py = _REF.sub(ref, py)
        parts = py.split('"')
        py = '"'.join(_NAME.sub(name, p) if i % 2 == 0 else p for i, p in enumerate(parts))
        py = py.replace("<>", "!=")
        py = re.sub(r"(?<![<>=!])=(?!=)", "==", py)
        py = py.replace("^", "**").replace("IF(", "_if(").replace("SUM(", "_sum(")
        env = {"_v": self.value, "_if": lambda c, a, b="": a if c else b,
               "_sum": lambda xs: sum(float(x) for x in xs)}
        try:
            return eval(py, {"__builtins__": {}}, env)  # noqa: S307 - own workbook, restricted env
        except ZeroDivisionError:
            raise ValueError(f"{where}: ={expr} divides by an empty/zero value - fill in the referenced cell")

    def records(self, sheet, required):
        """Rows as {header: value}; skips rows where any `required` column is empty
        (blank separator rows and group-title rows)."""
        ws = self.wb[sheet]
        headers = {str(ws.cell(1, c).value).strip(): c for c in range(1, ws.max_column + 1) if ws.cell(1, c).value}
        for h in required:
            if h not in headers:
                raise ValueError(f"sheet {sheet!r}: column {h!r} not found (have {list(headers)})")
        for r in range(2, ws.max_row + 1):
            if any(is_blank(ws.cell(r, headers[h]).value) for h in required):
                continue
            yield r, {h: self.value(sheet, r, c) for h, c in headers.items()}


def is_blank(v):
    return isinstance(v, Blank) or v is None or v == ""


def num(v, what):
    if is_blank(v) or isinstance(v, str):
        raise ValueError(f"missing number: {what} (got {v!r})")
    return float(v)


# --------------------------------------------------------------------------
# Input model
# --------------------------------------------------------------------------

KEYWORDS = ("DOWNLINK_DECISIONAL", "OBSERVATIONS", "WAIT_GROUND", "OFFLOAD")
# parameter keys read directly by this script (all others must be used by a formula)
CODE_KEYS = {"lunar_day", "start_min", "end_before_sunset", "t_margin", "sweep_step", "v_walk",
             "dl_rate_sci", "health_rate", "storage", "dl_threshold", "phase_end_flush", "wait_mode", "outage_dur", "outage_period", "outage_offset"}


def kbps_to_mbs(kbps):
    """kbit/s -> MB/s, with the workbook's 1024 convention (sizes are MiB)."""
    return kbps / 1024 / 8


def opt(v, default=0.0):
    return default if is_blank(v) else float(v)


def load_inputs(path):
    wb = Workbook(path)
    params = {v["key"]: v["value"] for _, v in wb.records("Parameters", ["key"])}

    modes = {}
    for _, v in wb.records("Modes", ["ID", "mode"]):
        modes[v["ID"]] = dict(id=v["ID"], name=v["mode"], actor=v["actor"], power=v["power (W)"],
                              duration=v["default duration (s)"],
                              per_run=opt(v["data per run (MB)"]), per_m=opt(v["data per m (MB)"]),
                              hk_kbps=opt(v["housekeeping (kbps)"]),
                              dtype="none" if is_blank(v["data type"]) else v["data type"])

    procs = defaultdict(list)
    for _, v in wb.records("Procedures", ["procedure", "mode / procedure"]):
        procs[v["procedure"]].append(dict(step=v["step"], item=v["mode / procedure"], name=v["name"],
                                          count=v["count"], override=v["duration override (s)"]))
    for p in procs.values():
        p.sort(key=lambda s: s["step"])

    activities, skipped = [], []
    for r, v in wb.records("Activity map", ["phase", "procedure"]):
        if v["procedure"] not in procs:
            skipped.append(v["science ID"])
            continue
        runs = v["runs (baseline)"]
        activities.append(dict(row=r, phase=int(v["phase"]), sid=v["science ID"], proc=v["procedure"],
                               role=str(v.get("role") or "").strip().lower(),
                               runs=0 if is_blank(runs) else int(round(float(runs))),
                               decision=str(v["extra decision point after each run"]).strip().lower() == "yes"))

    phases = {}
    for _, v in wb.records("Phases", ["phase", "name"]):
        phases[int(v["phase"])] = dict(name=v["name"], walked_m=num(v["walked distance (m)"],
                                                                     f"Phases walked distance, phase {v['phase']}"))

    checks = consistency_checks(wb, params, modes, procs, activities, skipped)
    return params, modes, procs, activities, phases, checks


def consistency_checks(wb, params, modes, procs, activities, skipped):
    """Unused items and science rows not covered - printed with every run."""
    out = []
    used_procs, stack = set(), [a["proc"] for a in activities]
    while stack:
        p = stack.pop()
        if p in used_procs:
            continue
        used_procs.add(p)
        stack += [s["item"] for s in procs[p] if s["item"] in procs]
    used_modes = {s["item"] for p in used_procs for s in procs[p]} | {params.get("wait_mode"), "OPS-12-b",
                                                                        "GND-01", "GND-02"}
    if x := sorted(set(procs) - used_procs):
        out.append(f"Procedures not used by the Activity map: {', '.join(x)}")
    if x := sorted(set(modes) - used_modes):
        labels = [f"{m} ({modes[m]['name']})" for m in x]
        out.append(f"Modes not used in any procedure: {', '.join(labels)}")
    if x := sorted({s["item"] for p in procs.values() for s in p} - set(modes) - set(procs) - set(KEYWORDS)):
        out.append(f"Procedure steps pointing to unknown modes/procedures: {', '.join(map(str, x))}")
    named = set()
    for _, _, f in wb.formulas():
        named |= wb._names_in(f[1:])
    if x := sorted(set(params) - named - CODE_KEYS):
        out.append(f"Parameters not used anywhere: {', '.join(x)}")
    mission = [v["ID"] for _, v in wb.records("Mission", ["ID"])]
    covered = {a["sid"] for a in activities}
    if x := [s for s in mission if s not in covered]:
        out.append(f"Mission rows without a simulated activity: {', '.join(x)}"
                   + (f" (marked not simulated: {', '.join(map(str, skipped))})" if skipped else ""))
    return out


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------

@dataclass
class Task:
    start_s: float
    duration_s: float
    phase: int
    science_id: str
    activity: str      # Activity map procedure this step belongs to
    procedure: str     # innermost procedure
    mode_id: str
    name: str
    power_w: float
    distance_m: float = 0.0
    data_mb: float = 0.0
    data_type: str = "none"

    @property
    def energy_wh(self):
        return self.power_w * self.duration_s / 3600.0


class Sim:
    def __init__(self, params, modes, procs, activities, phases, checks=(), outage_offset_h=None):
        self.p, self.modes, self.procs, self.acts, self.phases = params, modes, procs, activities, phases
        self.checks = list(checks)
        self.v_walk = num(params["v_walk"], "v_walk")
        self.dl_rate = kbps_to_mbs(num(params["dl_rate_sci"], "dl_rate_sci"))  # MB/s
        self.threshold = num(params["dl_threshold"], "dl_threshold")
        self.storage = num(params["storage"], "storage")
        self.flush_at_phase_end = str(params.get("phase_end_flush", "yes")).lower() == "yes"
        self.wait_mode = modes[params["wait_mode"]]
        # comms outage: outage_dur every outage_period, first one outage_offset after t = 0 (start of egress)
        self.out_period = num(params["outage_period"], "outage_period") * 3600
        self.out_dur = opt(params.get("outage_dur")) * 3600
        off = opt(params.get("outage_offset")) if outage_offset_h is None else outage_offset_h
        self.out_offset = off * 3600
        self.t = 0.0
        self.tasks = []
        self.health_mb = 0.0  # all housekeeping produced
        self.buffer = {"science": 0.0, "operator": 0.0, "health": 0.0}  # onboard, not yet sent
        self.buffer_trace = [(0.0, 0.0, 0.0, 0.0)]  # (t, science, operator, health)
        self.peak_buffer = 0.0
        self.decision_ready = None    # time the pending ground decision is back
        self.decision_unsent = False  # operator data could not go down (outage)
        self.ground_spans = []        # (start_s, duration_s, phase), parallel to the robot
        self.activity = ""

    # --- comms outage
    def _outage_window(self, t):
        """(start, end) of the outage containing t, else None."""
        if self.out_dur <= 0:
            return None
        s = self.out_offset + math.floor((t - self.out_offset) / self.out_period) * self.out_period
        return (s, s + self.out_dur) if s <= t < s + self.out_dur else None

    def _next_outage_start(self, t):
        if self.out_dur <= 0:
            return math.inf
        s = self.out_offset + math.floor((t - self.out_offset) / self.out_period) * self.out_period
        return s if s > t else s + self.out_period

    def _outage_overlap(self, a, b):
        if self.out_dur <= 0 or b <= a:
            return 0.0
        total, k = 0.0, math.floor((a - self.out_offset) / self.out_period)
        while True:
            s = self.out_offset + k * self.out_period
            if s >= b:
                return total
            total += max(0.0, min(b, s + self.out_dur) - max(a, s))
            k += 1

    def outages(self, t0, t1):
        """All outage windows overlapping [t0, t1] (for plotting)."""
        if self.out_dur <= 0:
            return []
        k, out = math.floor((t0 - self.out_offset) / self.out_period), []
        while (s := self.out_offset + k * self.out_period) < t1:
            if s + self.out_dur > t0:
                out.append((s, s + self.out_dur))
            k += 1
        return out

    # --- helpers
    def _walk_steps_in(self, proc):
        n = 0
        for s in self.procs[proc]:
            if s["item"] in self.procs:
                n += self._count(s) * self._walk_steps_in(s["item"])
            elif isinstance(s["override"], str) and s["override"].strip().lower() == "from phase":
                n += self._count(s)
        return n

    def _count(self, step):
        c = step["count"]
        return 1 if is_blank(c) else int(round(num(c, f"count of {step['item']}")))

    def _emit(self, phase, sid, proc, mode, duration, power=None, name=None, distance=0.0, data=0.0, dtype="none",
              drain=None):
        """Append one step. `data` is added and `drain` ({type: MB}) removed over
        the step, so the onboard-data trace changes linearly from its start to its end."""
        task = Task(self.t, duration, phase, sid, self.activity, proc, mode["id"], name or mode["name"],
                    mode["power"] if power is None else power, distance, data, dtype)
        if power is None and is_blank(task.power_w):
            raise ValueError(f"missing power for mode {mode['id']} ({mode['name']})")
        task.power_w = float(task.power_w)
        self.tasks.append(task)
        self._trace()  # level at the start of the step
        # housekeeping goes down continuously, except during an outage: then it is stored
        hk = kbps_to_mbs(mode.get("hk_kbps", 0.0))
        self.health_mb += hk * duration
        self.buffer["health"] += hk * self._outage_overlap(self.t, self.t + duration)
        self.t += duration
        if dtype in self.buffer and data:
            self.buffer[dtype] += data
        for k, mb in (drain or {}).items():
            self.buffer[k] = max(0.0, self.buffer[k] - mb)
        self._trace()  # level at the end of the step

    def _trace(self):
        b = self.buffer
        self.buffer_trace.append((self.t, b["science"], b["operator"], b["health"]))
        self.peak_buffer = max(self.peak_buffer, sum(b.values()))

    def _wait_link(self, phase, sid, proc):
        w = self._outage_window(self.t)
        if w:
            self._emit(phase, sid, proc, self.wait_mode, w[1] - self.t, name="Wait for link (comms outage)")

    def _downlink(self, phase, sid, proc, kinds, reason, wait=False):
        """Send the given onboard data. During an outage: wait for the link if
        `wait`, else leave the data onboard. A downlink is cut at the next outage
        start; with `wait` the rest goes after it. Returns True if all was sent."""
        mode = self.modes["OPS-12-b"]
        while (mb := sum(self.buffer[k] for k in kinds)) > 1e-9:
            if self._outage_window(self.t):
                if not wait:
                    return False
                self._wait_link(phase, sid, proc)
            send = min(mb, (self._next_outage_start(self.t) - self.t) * self.dl_rate)
            drain = {k: self.buffer[k] * send / mb for k in kinds}
            self._emit(phase, sid, proc, mode, send / self.dl_rate, name=f"Data downlink ({reason})", drain=drain)
            if send < mb - 1e-9 and not wait:
                return False
        return True

    def _start_ground_decision(self, phase):
        """Operator data is down: the ground decides in parallel, pausing during outages."""
        remaining, t = self._ground_time(), self.t
        while True:
            if w := self._outage_window(t):
                t = w[1]
            avail = self._next_outage_start(t) - t
            if remaining <= avail:
                t += remaining
                break
            remaining -= avail
            t += avail
        self.ground_spans.append((self.t, t - self.t, phase))
        self.decision_ready = t

    def _ground_time(self):
        return sum(num(self.modes[g]["duration"], f"{g} duration") for g in ("GND-01", "GND-02"))

    def _send_operator_data(self, phase, sid, proc, wait):
        if self._downlink(phase, sid, proc, ["operator"], "NAV + operator image", wait=wait):
            self._start_ground_decision(phase)
            self.decision_unsent = False
        else:
            self.decision_unsent = True

    def _wait_ground(self, phase, sid, proc):
        if self.decision_unsent:
            self._send_operator_data(phase, sid, proc, wait=True)
        if self.decision_ready is not None and self.decision_ready > self.t:
            self._emit(phase, sid, proc, self.wait_mode, self.decision_ready - self.t,
                       name="Wait for ground decision")
        self.decision_ready = None

    def _decision(self, phase, sid, proc):
        """Blocking decision point: send operator data, wait for the full ground decision."""
        self._send_operator_data(phase, sid, proc, wait=True)
        self._wait_ground(phase, sid, proc)

    def _run_observations(self, phase, obs, seg_m):
        for a in obs:
            self.activity = a["proc"]
            self._run_proc(phase, a["sid"], a["proc"], seg_m)
            if a["decision"]:
                self._decision(phase, a["sid"], a["proc"])
            if self.buffer["science"] >= self.threshold:
                self._downlink(phase, a["sid"], a["proc"], ["science", "operator", "health"], "threshold")

    # --- expansion
    def _run_proc(self, phase, sid, proc, seg_m, obs=()):
        for step in self.procs[proc]:
            n = self._count(step)
            if n <= 0:
                continue
            item = step["item"]
            if item in self.procs:
                for _ in range(n):
                    self._run_proc(phase, sid, item, seg_m)
                continue
            if item == "DOWNLINK_DECISIONAL":
                self._send_operator_data(phase, sid, proc, wait=False)
                continue
            if item == "OBSERVATIONS":
                station = self.activity
                self._run_observations(phase, obs, seg_m)
                self.activity = station
                continue
            if item == "WAIT_GROUND":
                self._wait_ground(phase, sid, proc)
                continue
            if item == "OFFLOAD":
                self._downlink(phase, sid, proc, ["science", "operator", "health"], "offload")
                continue
            if item not in self.modes:
                raise ValueError(f"procedure {proc}: unknown mode or procedure {item!r}")
            mode = self.modes[item]
            ov = step["override"]
            label = step["name"] if not is_blank(step["name"]) else mode["name"]
            for _ in range(n):
                distance = 0.0
                if isinstance(ov, str) and ov.strip().lower() == "from phase":
                    distance = seg_m
                    dur = distance / self.v_walk
                elif not is_blank(ov):
                    dur = num(ov, f"{proc} step {step['step']} duration override")
                    if isinstance(mode["duration"], str) and "distance" in mode["duration"]:
                        distance = dur * self.v_walk  # short walk given as time (side step, PSR step)
                else:
                    dur = num(mode["duration"], f"default duration of {item} ({mode['name']})")
                if distance > 0 and self._outage_overlap(self.t, self.t + dur) > 0:
                    # no walking into a comms outage: wait until the link is back
                    w = self._outage_window(self.t) or self._outage_window(self._next_outage_start(self.t))
                    self._emit(phase, sid, proc, self.wait_mode, w[1] - self.t,
                               name="Wait for link (no walking during outage)")
                data = mode["per_run"] + mode["per_m"] * distance
                self._emit(phase, sid, proc, mode, dur, name=label, distance=distance,
                           data=data, dtype=mode["dtype"])

    def _is_station(self, proc):
        return any(s["item"] == "OBSERVATIONS" for s in self.procs[proc])

    def _assign(self, acts):
        """Split a phase's Activity map rows into: rows run before the first
        station, and one observation list per station run (spread evenly; same
        science ID first, else the phase's largest station row)."""
        first = next((i for i, a in enumerate(acts) if self._is_station(a["proc"])), None)
        if first is None:
            return [x for a in acts for x in [a] * a["runs"]], []
        pre = [x for a in acts[:first] for x in [a] * a["runs"]]
        st_rows = [a for a in acts if self._is_station(a["proc"])]
        slots = {id(a): [[] for _ in range(a["runs"])] for a in st_rows}
        largest = max(st_rows, key=lambda a: a["runs"])
        for a in acts[first:]:
            if self._is_station(a["proc"]) or a["runs"] <= 0 or a["role"] == "after stations":
                continue
            targets = [s for s in st_rows if s["sid"] == a["sid"]] or [largest]
            stations = [slot for s in targets for slot in slots[id(s)]]
            for i in range(a["runs"]):
                stations[int((i + 0.5) * len(stations) / a["runs"])].append(a)
        return pre, [(s, obs) for s in st_rows for obs in slots[id(s)]]

    def run(self):
        everything = ["science", "operator", "health"]
        for ph in sorted({a["phase"] for a in self.acts}):
            acts = [a for a in self.acts if a["phase"] == ph]
            n_walk = sum(a["runs"] * self._walk_steps_in(a["proc"]) for a in acts)
            seg_m = self.phases[ph]["walked_m"] / n_walk if n_walk else 0.0
            if not n_walk and self.phases[ph]["walked_m"] > 0:
                print(f"WARNING: phase {ph} has {self.phases[ph]['walked_m']:.0f} m to walk but no walking steps")
            post = [x for a in acts if a["role"] == "after stations" for x in [a] * a["runs"]]
            pre, stations = self._assign([a for a in acts if a["role"] != "after stations"])
            self._run_observations(ph, pre, seg_m)
            for st, obs in stations:
                self.activity = st["proc"]
                self._run_proc(ph, st["sid"], st["proc"], seg_m, obs)
                if st["decision"]:
                    self._decision(ph, st["sid"], st["proc"])
            self._wait_ground(ph, "", "PHASE_END")
            self._run_observations(ph, post, seg_m)  # e.g. locomotion demonstration
            self.activity = "PHASE_END"
            if self.flush_at_phase_end:
                self._downlink(ph, "", "PHASE_END", everything, "phase end")  # skipped during an outage
        last = max(self.phases)
        self._downlink(last, "", "MISSION_END", everything, "end of mission", wait=True)
        return self


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def window(p):
    start_min = num(p["start_min"], "start_min")
    end_max = num(p["lunar_day"], "lunar_day") - num(p["end_before_sunset"], "end_before_sunset")
    return start_min, end_max, num(p["t_margin"], "t_margin") / 100.0


def offset_sweep(inputs, params):
    """Mission duration for first-outage offsets over one outage period."""
    step = num(params["sweep_step"], "sweep_step")
    period = num(params["outage_period"], "outage_period")
    if opt(params.get("outage_dur")) <= 0:
        return []
    out, off = [], 0.0
    while off < period - 1e-9:
        s = Sim(*inputs, outage_offset_h=off).run()
        waits = sum(t.duration_s for t in s.tasks if t.name.startswith("Wait")) / 3600
        out.append((off, s.t / 3600, waits))
        off += step
    return out


def summarize(sim, sweep):
    p = sim.p
    lines = []
    T = sim.t / 3600.0
    start_min, end_max, margin = window(p)
    avail = end_max - start_min
    required = T * (1 + margin)

    lines.append("SOUTH POLE BASELINE - FEASIBILITY SUMMARY")
    lines.append("=" * 64)
    lines.append(f"Mission duration:      {T:8.1f} h  ({T / 24:.1f} Earth days), first outage "
                 f"{sim.out_offset / 3600:g} h after start of egress")
    lines.append(f"With {margin:.0%} time margin:  {required:8.1f} h")
    lines.append(f"Window:                {avail:8.1f} h  ({start_min:.1f} h to {end_max:.1f} h after sunrise)")
    lines.append(f"Result:                {'FITS' if required <= avail else 'DOES NOT FIT'}  "
                 f"(slack {avail - required:+.1f} h)")
    if sweep:
        worst = max(sweep, key=lambda s: s[1])
        best = min(sweep, key=lambda s: s[1])
        lines.append(f"Outage timing sweep:   {best[1]:.1f} h to {worst[1]:.1f} h "
                     f"(worst: first outage {worst[0]:g} h after egress start) -> "
                     f"{'FITS' if worst[1] * (1 + margin) <= avail else 'DOES NOT FIT'} in the worst case")
    lines.append("")

    E = sum(t.energy_wh for t in sim.tasks)
    lines.append(f"Energy total:          {E:8.0f} Wh")
    lines.append(f"Peak power:            {max(t.power_w for t in sim.tasks):8.1f} W")
    lines.append(f"Average power:         {E / T if T else 0:8.1f} W")
    lines.append(f"Distance walked:       {sum(t.distance_m for t in sim.tasks):8.1f} m")
    lines.append("")

    sci = sum(t.data_mb for t in sim.tasks if t.data_type == "science")
    op = sum(t.data_mb for t in sim.tasks if t.data_type == "operator")
    hk_max = max(m["hk_kbps"] for m in sim.modes.values())
    share = num(p["health_rate"], "health_rate")
    lines.append(f"Science data:          {sci:8.1f} MB  (images + GPR, sent at the station offload)")
    lines.append(f"Operator data:         {op:8.1f} MB  (NAV + compressed operator image, sent for every ground decision)")
    lines.append(f"Housekeeping data:     {sim.health_mb:8.1f} MB  (max {hk_max:g} kbps on a {share:g} kbps share: "
                 f"{'OK' if hk_max <= share else 'TOO LOW'}; stored onboard during outages)")
    lines.append(f"Science link rate:     {num(p['dl_rate_sci'], 'dl_rate_sci'):8.0f} kbps")
    lines.append(f"Peak onboard:          {sim.peak_buffer:8.1f} MB of {sim.storage:.0f} MB"
                 + ("  ** EXCEEDS STORAGE **" if sim.peak_buffer > sim.storage else ""))
    t_dl = sum(t.duration_s for t in sim.tasks if t.mode_id == "OPS-12-b") / 3600
    t_wait_g = sum(t.duration_s for t in sim.tasks if t.name == "Wait for ground decision") / 3600
    t_wait_l = sum(t.duration_s for t in sim.tasks if t.name.startswith("Wait for link")) / 3600
    lines.append(f"Ground decisions:      {len(sim.ground_spans):8d}     ({t_wait_g:.1f} h waiting for the decision)")
    lines.append(f"Waiting for the link:  {t_wait_l:8.1f} h  (comms outages)")
    lines.append(f"Downlink time:         {t_dl:8.1f} h")
    lines.append("")

    lines.append(f"{'Phase':<38}{'time h':>8}{'energy Wh':>11}{'walk m':>8}{'sci MB':>8}")
    for ph in sorted(sim.phases):
        ts = [t for t in sim.tasks if t.phase == ph]
        if ts:
            lines.append(f"{ph} {sim.phases[ph]['name']:<36}{sum(t.duration_s for t in ts) / 3600:8.1f}"
                         f"{sum(t.energy_wh for t in ts):11.0f}{sum(t.distance_m for t in ts):8.1f}"
                         f"{sum(t.data_mb for t in ts if t.data_type == 'science'):8.1f}")
    lines.append("")

    by_mode = defaultdict(lambda: [0.0, 0.0, 0])
    for t in sim.tasks:
        key = t.name if t.name.startswith("Wait") else sim.modes[t.mode_id]["name"]
        by_mode[key][0] += t.duration_s / 3600
        by_mode[key][1] += t.energy_wh
        by_mode[key][2] += 1
    lines.append(f"{'Mode':<44}{'time h':>8}{'energy Wh':>11}{'count':>8}")
    for k, (h, e, n) in sorted(by_mode.items(), key=lambda kv: -kv[1][0]):
        lines.append(f"{k[:43]:<44}{h:8.2f}{e:11.0f}{n:8d}")
    lines.append("")

    runs = defaultdict(int)
    for a in sim.acts:
        runs[a["proc"]] += a["runs"]
    by_act = defaultdict(lambda: [0.0, 0])
    for t in sim.tasks:
        by_act[t.activity][0] += t.duration_s / 3600
        by_act[t.activity][1] += t.mode_id == "OPS-17"
    lines.append(f"{'Activity (Activity map procedure)':<36}{'runs':>6}{'SciCAM img':>12}{'time h':>8}")
    for k, (h, n) in sorted(by_act.items(), key=lambda kv: -kv[1][0]):
        lines.append(f"{k:<36}{runs.get(k, ''):>6}{n:12d}{h:8.2f}")
    lines.append("")

    img = defaultdict(lambda: [0, 0, 0])  # science ID -> SciCAM full res, operator images, NAV images
    for t in sim.tasks:
        if t.mode_id in ("OPS-17", "NEW-06", "OPS-10"):
            img[t.science_id or "-"][("OPS-17", "NEW-06", "OPS-10").index(t.mode_id)] += 1
    lines.append(f"{'Images by science ID':<22}{'SciCAM full res':>16}{'operator img':>14}{'NAV img':>9}")
    for sid in sorted(img):
        a, b, c = img[sid]
        lines.append(f"{sid:<22}{a:16d}{b:14d}{c:9d}")
    tot = [sum(v[i] for v in img.values()) for i in range(3)]
    lines.append(f"{'total':<22}{tot[0]:16d}{tot[1]:14d}{tot[2]:9d}")
    lines.append("  (station images - full res + operator + NAV after every walk - count under the station's science ID)")
    lines.append("")

    if sweep:
        lines.append("Outage timing sweep (first outage after start of egress)")
        for off, dur, w in sweep:
            lines.append(f"  first outage at {off:5.1f} h -> mission {dur:6.1f} h, waiting {w:5.1f} h, "
                         f"slack with margin {avail - dur * (1 + margin):+6.1f} h")
        lines.append("")
    lines.append("Workbook checks")
    lines += [f"  - {c}" for c in sim.checks] or ["  all items used, all science rows covered"]
    return "\n".join(lines)


# --- plots ------------------------------------------------------------------

INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
C_ROBOT, C_WAIT, C_GROUND, C_LINK = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
ROWS = [
    ("Egress / checkout", lambda t: t.mode_id in ("NEW-04", "NEW-01"), C_ROBOT),
    ("Locomotion demo", lambda t: t.mode_id == "NEW-07", C_ROBOT),
    ("Walking", lambda t: t.distance_m > 0, C_ROBOT),
    ("Turning", lambda t: t.mode_id == "OPS-23", C_ROBOT),
    ("Standing (dust / wait)", lambda t: t.mode_id == "OPS-07", C_ROBOT),
    ("Imaging + compression", lambda t: t.mode_id in ("OPS-10", "OPS-17", "NEW-05", "NEW-06"), C_ROBOT),
    ("Tilting / pedipulation", lambda t: t.mode_id in ("OPS-04", "OPS-05", "NEW-03"), C_ROBOT),
    ("Nap", lambda t: t.mode_id == "OPS-20", C_ROBOT),
    ("Downlink", lambda t: t.mode_id == "OPS-12-b", C_LINK),
    ("Waiting", lambda t: t.name.startswith("Wait"), C_WAIT),
    ("Ground decision", None, C_GROUND),
]


def _style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)


def plot_phases(sim, path, title):
    """One panel per phase, all on the same time scale (hours after sunrise).
    Egress + checkout is shown as a short broken-axis block so it does not
    swamp phase 1."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    BREAK_W = 0.8 * 3600  # display width of a broken-out block (s)
    h0 = num(sim.p["start_min"], "start_min")
    phases = sorted({t.phase for t in sim.tasks})

    def warp_for(ts):
        blk = [t for t in ts if t.activity == "CHECKOUT"]
        breaks = [(blk[0].start_s, blk[-1].start_s + blk[-1].duration_s)] if blk else []

        def f(t):  # real time (s) -> display time (s)
            x = t
            for a, b in breaks:
                if t >= b:
                    x -= (b - a) - BREAK_W
                elif t > a:
                    x = a + (t - a) / (b - a) * BREAK_W
            return x
        return f, breaks

    panels = []
    for ph in phases:
        ts = [t for t in sim.tasks if t.phase == ph]
        f, breaks = warp_for(ts)
        t0, t1 = ts[0].start_s, ts[-1].start_s + ts[-1].duration_s
        panels.append((ph, ts, f, breaks, t0, t1))
    span = max(f(t1) - f(t0) for _, _, f, _, t0, t1 in panels)

    fig, axes = plt.subplots(len(phases), 1, figsize=(16, 2.9 * len(phases)), facecolor=SURFACE)
    fig.suptitle(title, x=0.01, ha="left", color=INK, fontsize=13, fontweight="bold")
    for ax, (ph, ts, f, breaks, t0, t1) in zip(axes, panels):
        _style(ax)
        X = lambda t: f(t) / 3600  # display hours
        ax.set_xlim(X(t0), X(t0) + span / 3600)  # same width and scale for every phase
        for a, b in sim.outages(t0, t1):
            a, b = max(a, t0), min(b, t1)
            ax.axvspan(X(a), X(b), color=GRID, alpha=0.7, lw=0, zorder=0)
            visible = [(a, b)]  # label only the part of an outage outside a broken-out block
            for ba, bb in breaks:
                visible = [(max(x, bb), y) for x, y in visible if y > bb] + [(x, min(y, ba)) for x, y in visible if x < ba]
            if any(y - x > 600 for x, y in visible):
                x0 = min(x for x, y in visible if y - x > 600)
                ax.text(X(x0), -0.9, " comms outage", color=MUTED, fontsize=8, va="bottom")
        for a, b in breaks:
            ax.axvspan(X(a), X(b), facecolor="none", edgecolor=MUTED, hatch="///", lw=0, alpha=0.5, zorder=3)
            ax.text((X(a) + X(b)) / 2, len(ROWS) / 2 - 0.5, f"egress + checkout {(b - a) / 3600:.0f} h",
                    color=INK, fontsize=8, ha="center", va="center", rotation=90, zorder=4,
                    bbox=dict(facecolor=SURFACE, edgecolor="none", pad=1.5))
        for t in ts:  # fine line at every station arrival
            if t.activity == "WALK_TO_STATION" and t.distance_m > 0 and t.procedure == "WALK_TO_STATION":
                ax.axvline(X(t.start_s), color=MUTED, lw=0.4, alpha=0.5, zorder=1)
        for i, (label, pred, color) in enumerate(ROWS):
            if pred is None:
                raw = [(s0, d) for s0, d, p in sim.ground_spans if p == ph]
            else:
                raw = [(t.start_s, t.duration_s) for t in ts if pred(t)]
            spans = [(X(s0), X(s0 + d) - X(s0)) for s0, d in raw]
            if spans:
                ax.broken_barh(spans, (i - 0.32, 0.64), facecolors=color, linewidth=0, zorder=2)
        # ticks in real hours after sunrise, every 2 h, skipping broken-out blocks
        ticks, k = [], math.ceil((h0 + t0 / 3600) / 2) * 2
        while (k - h0) * 3600 <= t0 + span + sum(b - a for a, b in breaks):
            tr = (k - h0) * 3600
            if not any(a <= tr <= b for a, b in breaks):
                ticks.append((X(tr), f"{k:g}"))
            k += 2
        ax.set_xticks([x for x, _ in ticks], [l for _, l in ticks])
        ax.set_yticks(range(len(ROWS)), [r[0] for r in ROWS], color=INK, fontsize=8.5)
        ax.set_ylim(len(ROWS) - 0.4, -1)
        n_st = sum(1 for t in ts if t.activity == "WALK_TO_STATION" and t.distance_m > 0
                   and t.procedure == "WALK_TO_STATION")
        head = f"Phase {ph}: {sim.phases[ph]['name']}  ·  {(t1 - t0) / 3600:.1f} h  ·  {n_st} stations"
        ax.set_title(head, loc="left", color=INK, fontsize=10, pad=4)
        ax.grid(axis="x", color=GRID, lw=0.6)
    axes[-1].set_xlabel("hours after sunrise   (all phases on the same scale; thin vertical lines = arrival at a station)",
                        color=INK, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=140, facecolor=SURFACE)
    plt.close(fig)


def plot_overview(sim, path, title):
    """Whole mission: onboard data by type, phases and outages."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    h0 = num(sim.p["start_min"], "start_min")
    H = lambda s: h0 + s / 3600
    fig, ax = plt.subplots(figsize=(16, 4.2), facecolor=SURFACE)
    _style(ax)
    for a, b in sim.outages(0, sim.t):
        ax.axvspan(H(a), H(min(b, sim.t)), color=GRID, alpha=0.7, lw=0)
    top = max(sim.peak_buffer * 1.35, 1.0)
    for ph in sorted(sim.phases):
        ts = [t for t in sim.tasks if t.phase == ph]
        if ts:
            ax.axvline(H(ts[0].start_s), color=MUTED, lw=0.8)
            ax.text(H(ts[0].start_s), top * 0.97, f" phase {ph}", color=INK, fontsize=9, va="top")
    xs = [H(r[0]) for r in sim.buffer_trace]
    sci = [r[1] for r in sim.buffer_trace]
    op = [r[1] + r[2] for r in sim.buffer_trace]
    tot = [r[1] + r[2] + r[3] for r in sim.buffer_trace]
    ax.fill_between(xs, 0, sci, color=C_ROBOT, lw=0, label="science")
    ax.fill_between(xs, sci, op, color=C_GROUND, lw=0, label="operator")
    ax.fill_between(xs, op, tot, color=C_WAIT, lw=0, label="housekeeping (outage)")
    if sim.storage <= top:
        ax.axhline(sim.storage, color="#e34948", lw=1, ls="--")
    ax.set_ylim(0, top)
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylabel("onboard data (MB)", color=INK, fontsize=9)
    ax.set_xlabel("hours after sunrise   (grey = comms outage)", color=INK, fontsize=9)
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), frameon=False, fontsize=8, ncol=3, borderaxespad=0)
    ax.set_title(f"{title}  ·  peak {sim.peak_buffer:.0f} MB of {sim.storage:.0f} MB storage",
                 loc="left", color=INK, fontsize=12, fontweight="bold", pad=18)
    fig.tight_layout()
    fig.savefig(path, dpi=140, facecolor=SURFACE)
    plt.close(fig)

CATEGORICAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
OTHER = "#b9b8b2"


def time_category(t):
    if t.mode_id in ("NEW-04", "NEW-01"):
        return "Egress + checkout"
    if t.mode_id == "NEW-07":
        return "Locomotion demo"
    if t.name.startswith("Wait for link"):
        return "Waiting: comms outage"
    if t.name.startswith("Wait for ground"):
        return "Waiting: ground decision"
    if t.distance_m > 0:
        return "Walking"
    return {"OPS-12-b": "Downlink", "OPS-23": "Turning", "OPS-07": "Standing (dust / waits)", "OPS-20": "Nap",
            "OPS-10": "Imaging", "OPS-17": "Imaging", "NEW-06": "Imaging", "NEW-05": "Imaging",
            "NEW-03": "Pedipulation", "OPS-04": "Tilting", "OPS-05": "Tilting"}.get(t.mode_id, "Other")


def plot_stats(sim, path, title, sweep):
    """One page for the team: headline numbers, where the time goes, energy per mode."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p = sim.p
    T = sim.t / 3600
    start_min, end_max, margin = window(p)
    avail = end_max - start_min
    E = sum(t.energy_wh for t in sim.tasks)
    sci = sum(t.data_mb for t in sim.tasks if t.data_type == "science")
    worst = max((s[1] for s in sweep), default=T)
    tiles = [
        ("Mission duration", f"{T:.1f} h", f"{T / 24:.1f} Earth days; worst outage timing {worst:.1f} h"),
        (f"With {margin:.0%} time margin", f"{T * (1 + margin):.0f} h", f"window {avail:.0f} h ({start_min:.0f}–{end_max:.0f} h after sunrise)"),
        ("Slack", f"{avail - worst * (1 + margin):+.0f} h", "worst case, after margin"),
        ("Energy", f"{E:,.0f} Wh", f"average {E / T:.0f} W, peak {max(t.power_w for t in sim.tasks):.0f} W"),
        ("Distance walked", f"{sum(t.distance_m for t in sim.tasks):.0f} m", "incl. tortuosity and side steps"),
        ("SciCAM images", f"{sum(t.mode_id == 'OPS-17' for t in sim.tasks)}",
         f"full resolution, + {sum(t.mode_id == 'NEW-06' for t in sim.tasks)} operator images"),
        ("Science data", f"{sci:,.0f} MB", f"peak onboard {sim.peak_buffer:.0f} of {sim.storage:.0f} MB"),
        ("Downlink time", f"{sum(t.duration_s for t in sim.tasks if t.mode_id == 'OPS-12-b') / 3600:.1f} h",
         f"science link {num(p['dl_rate_sci'], 'dl_rate_sci'):.0f} kbps"),
        ("Ground decisions", f"{len(sim.ground_spans)}", "one per station + go/no-go points"),
        ("Waiting", f"{sum(t.duration_s for t in sim.tasks if t.name.startswith('Wait')) / 3600:.1f} h",
         "comms outages + ground decisions"),
    ]

    fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
    fig.suptitle(title, x=0.02, y=0.975, ha="left", color=INK, fontsize=14, fontweight="bold")
    for i, (label, value, sub) in enumerate(tiles):  # KPI row: 2 x 5 stat tiles
        ax = fig.add_axes([0.03 + (i % 5) * 0.19, 0.81 - (i // 5) * 0.125, 0.175, 0.11])
        ax.set_axis_off()
        ax.add_patch(plt.Rectangle((0, 0), 1, 1, transform=ax.transAxes, facecolor="#f2f1ee", edgecolor="none"))
        ax.text(0.06, 0.78, label, color=MUTED, fontsize=10, transform=ax.transAxes)
        ax.text(0.06, 0.38, value, color=INK, fontsize=21, fontweight="semibold", transform=ax.transAxes)
        ax.text(0.06, 0.1, sub, color=MUTED, fontsize=7.8, transform=ax.transAxes)

    # time split (pie, as requested): <= 7 named slices, the rest folded into "Other"
    by_cat = defaultdict(float)
    for t in sim.tasks:
        by_cat[time_category(t)] += t.duration_s / 3600
    ranked = sorted(by_cat.items(), key=lambda kv: -kv[1])
    named, rest = ranked[:7], ranked[7:]
    if rest:
        named.append(("Other:\n" + ", ".join(k.lower() for k, _ in rest), sum(v for _, v in rest)))
    colors = CATEGORICAL[:len(ranked[:7])] + ([OTHER] if rest else [])
    ax = fig.add_axes([0.02, 0.05, 0.22, 0.5])
    wedges, _ = ax.pie([v for _, v in named], colors=colors, startangle=90, counterclock=False,
                       wedgeprops=dict(width=0.999, edgecolor=SURFACE, linewidth=2))
    ax.set_title(f"Where the {T:.0f} h go", loc="left", color=INK, fontsize=12, fontweight="bold")
    ax.legend(wedges, [f"{k}  ·  {v:.1f} h  ({v / T:.0%})" for k, v in named], loc="center left",
              bbox_to_anchor=(1.02, 0.5), frameon=False, fontsize=9, labelcolor=INK, ncol=1)
    for w, (k, v) in zip(wedges, named):
        if v / T >= 0.05:
            ang = math.radians((w.theta1 + w.theta2) / 2)
            ax.text(0.68 * math.cos(ang), 0.68 * math.sin(ang), f"{v / T:.0%}", ha="center", va="center",
                    color="white", fontsize=10, fontweight="bold")

    # energy per mode: ranked horizontal bars, one hue
    by_mode = defaultdict(float)
    for t in sim.tasks:
        key = ("Wait for link (comms outage)" if t.name.startswith("Wait for link") else
               t.name if t.name.startswith("Wait") else sim.modes[t.mode_id]["name"])
        by_mode[key] += t.energy_wh
    items = [kv for kv in sorted(by_mode.items(), key=lambda kv: -kv[1]) if kv[1] >= 0.5]
    ax = fig.add_axes([0.66, 0.05, 0.3, 0.5])
    _style(ax)
    ys = range(len(items))
    ax.barh(ys, [v for _, v in items], color=C_ROBOT, height=0.62)
    for y, (k, v) in zip(ys, items):
        ax.text(v + E * 0.004, y, f"{v:,.0f} Wh  ({v / E:.0%})", va="center", color=INK, fontsize=8.5)
    ax.set_yticks(list(ys), [k if len(k) <= 40 else k[:38] + "…" for k, _ in items], color=INK, fontsize=9)
    ax.invert_yaxis()
    ax.set_xlim(0, max(v for _, v in items) * 1.25)
    ax.grid(axis="x", color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.set_xlabel("energy (Wh)", color=INK, fontsize=9)
    ax.set_title(f"Energy per mode  ·  {E:,.0f} Wh in total", loc="left", color=INK, fontsize=12, fontweight="bold")
    fig.savefig(path, dpi=140, facecolor=SURFACE)
    plt.close(fig)


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_WORKBOOK
    inputs = load_inputs(path)
    sim = Sim(*inputs).run()
    sweep = offset_sweep(inputs, sim.p)
    report = summarize(sim, sweep)
    print(report)

    out = OUT_ROOT / f"run_{datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.txt").write_text(f"Input: {path}\n\n{report}\n")
    h0 = num(sim.p["start_min"], "start_min")
    with open(out / "timeline.csv", "w", newline="") as f:
        fields = ["start_h_after_sunrise"] + list(asdict(sim.tasks[0]).keys()) + ["energy_wh"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for t in sim.tasks:
            w.writerow({"start_h_after_sunrise": round(h0 + t.start_s / 3600, 4), **asdict(t),
                        "energy_wh": round(t.energy_wh, 4)})
    E = sum(t.energy_wh for t in sim.tasks)
    plot_phases(sim, out / "timeline_phases.png", f"South Pole baseline - {sim.t / 3600:.1f} h, {E:.0f} Wh ({path.name})")
    plot_overview(sim, out / "data_overview.png", f"Onboard data - {path.name}")
    plot_stats(sim, out / "stats.png", f"South Pole baseline - key numbers ({path.name})", sweep)
    print(f"\nOutputs: {out}")


if __name__ == "__main__":
    main()
