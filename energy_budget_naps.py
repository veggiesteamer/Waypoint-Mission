"""
Re-size every nap in a planned mission from an actual energy budget,
replacing the flat stand-in the planner uses (mission_ops_config.py's
nap_mode="constant" / nap_constant_min, or the nap_curve).

This is a *standalone post-processor*, deliberately not wired into
conops_planner: it takes a finished run under experiments/missions/, runs
a per-minute battery + stored-heat simulation over its rover timeline, and
lets each "nap" (OPS-20) task run only as long as it actually needs to --
until the battery is back to full and any heat built up over the day has
been dumped -- bounded by [NAP_MIN_MIN, NAP_MAX_MIN]. Everything after a
nap shifts by the difference, so the mission end moves too.

It is the Energy_Budget spreadsheet model (Timeline - PWR00.xlsx +
missionEnvelopeMCR.xlsx + a Parameters.py module) reduced to what this
repo can supply on its own:

  * Environment (per minute: Sun incidence on the array, Sun-Moon
    distance, dust, sunlight) comes from the "Timestep Timeline" tab of
    Timeline - PWR00.xlsx -- it is a function of absolute time in the
    lunation, unchanged by how long the rover naps, so the planned run's
    own nap lengths don't matter here.
  * The thermal dissipation envelope (min/max W the radiators must shed
    at each point in the day) is read straight from a previous full
    Energy_Budget run, "Energy budget results.csv" -- its
    "Dissipation min/max [W]" columns already are missionEnvelopeMCR.xlsx
    projected onto the timeline, so we don't need the envelope workbook
    itself. If that CSV is missing, the heat term is dropped and naps end
    on the battery criterion alone.
  * The EPS constants that were in Parameters.py (harness/PCDU/battery
    losses, quiescent draws, system margin, battery capacity, and the
    combined solar luminosity x cell efficiency x cell area term) were
    back-solved from that same CSV's intermediate columns and are pinned
    in EPS/DUST_* below. Verified to reproduce the CSV's "Power
    generation [W]" to 1e-14 W. Replace with the real Parameters.py
    values if you get them.

Per-task rover power is taken from the run's own report
(activity_energy_wh / activity_breakdown_h per activity name), so it
matches the planner's numbers exactly rather than the spreadsheet's
power matrix.

Outputs, next to the run's results.json:
  * Timeline_EnergyBudget.xlsx  -- the rover activity sequence with
    re-sized naps and shifted start/end times, plus a per-minute log
  * console summary comparing planned vs computed nap statistics
  * matplotlib figures (nap length over the mission, battery SoC,
    dissipation vs envelope)
"""
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------- config
# Which run to post-process -- None picks the most recent one, same
# convention as analyze_run.py / view_mission_playback.py.
RUN_ID = None
MISSIONS_DIR = Path("experiments/missions")

DOWNLOADS = Path.home() / "Downloads"
# "Timestep Timeline" tab supplies the per-minute environment.
PWR00_XLSX = DOWNLOADS / "Timeline - PWR00.xlsx"
# A previous full Energy_Budget run -- supplies the dissipation envelope.
# Set to None (or point at a missing file) to drop the heat term.
REFERENCE_CSV = DOWNLOADS / "Energy budget results.csv"

# Nap length bounds, minutes. The floor is the spreadsheet model's own
# 16-min minimum; the ceiling stops the battery criterion from looping
# forever once the mission has overrun into lunar dusk (no more
# generation -> battery never refills).
NAP_MIN_MIN = 16
NAP_MAX_MIN = 180

# Absolute minute in the "Timestep Timeline" that the run's t=0 (landing,
# start of Egress) lines up with. None -> Sunrise + mission_start_lunar_hour
# (from the run's config) so the Sun geometry matches the planner's clock.
ENV_OFFSET_MIN = None

# Solar generation is treated as available only from the end of this task
# onward (before that the array is stowed / the rover is still on the
# lander). "egress" or "commissioning".
GENERATION_STARTS_AFTER = "egress"

# ---- EPS constants, back-solved from REFERENCE_CSV (see module docstring).
# Swap in the real Parameters.py values here if you obtain them.
EPS = dict(
    harness_losses_pct=3.0,          # POW_105
    pcdu_conversion_eff_pct=95.0,    # POW_106
    pcdu_distribution_eff_pct=92.0,  # POW_107
    pcdu_quiescent_w=3.0,            # POW_108
    battery_quiescent_w=1.0,         # POW_303
    battery_eff_pct=86.0,            # POW_302
    system_margin_pct=20.0,          # SYS_004
    battery_capacity_wh=306.0,       # POW_300
    eol_battery_capacity_wh=306.0,   # EOL_battery_capacity
    solar_lum_eff_area=3.138924016800e25,  # ENV_005 * POW_204 * total_cell_area
)
# Relative power output vs. dust: min(1, a*exp(b*dust_mg_cm2))
DUST_A, DUST_B = 0.988294321, -0.211764707

NAP_NAMES = {"nap"}
NAP_IDS = {"OPS-20"}


# ------------------------------------------------------------- load run
def load_run():
    index = json.loads((MISSIONS_DIR / "index.json").read_text())
    if not index:
        raise ValueError(f"no runs in {MISSIONS_DIR / 'index.json'}")
    run_dir = Path(index[-1]["run_dir"]) if RUN_ID is None else MISSIONS_DIR / RUN_ID
    config = json.loads((run_dir / "config.json").read_text())
    results = json.loads((run_dir / "results.json").read_text())
    return run_dir, config, results


def per_name_power(report):
    """Average W for each activity name, from the run's own totals."""
    breakdown = report["activity_breakdown_h"]
    energy = report["activity_energy_wh"]
    return {name: (energy[name] / breakdown[name] if breakdown[name] else 0.0)
            for name in breakdown}


# ----------------------------------------------------- environment table
def _read_timestep_timeline():
    """The 'Timestep Timeline' tab, cached as a pickle next to the workbook
    (reading the ~18 MB .xlsx is most of this script's runtime)."""
    cache = PWR00_XLSX.with_suffix(".timestep.pkl")
    if cache.exists() and cache.stat().st_mtime >= PWR00_XLSX.stat().st_mtime:
        return pd.read_pickle(cache)
    tt = pd.read_excel(PWR00_XLSX, "Timestep Timeline")
    try:
        tt.to_pickle(cache)
    except OSError as exc:  # read-only dir, ...
        print(f"  (could not cache Timestep Timeline: {exc})")
    return tt


def load_environment():
    tt = _read_timestep_timeline()
    sa_col = next(c for c in tt.columns if str(c).startswith("Sun incidence on SA"))
    minute = tt["Time [min]"].to_numpy(dtype=float)
    env = {
        "minute": minute,
        "sun_incidence_sa_deg": tt[sa_col].to_numpy(dtype=float),
        "sun_moon_dist_m": tt["Sun-Moon distance [m]"].to_numpy(dtype=float),
        "dust_mg_cm2": tt["Dust accumulation [mg/cm2]"].to_numpy(dtype=float),
        "sunlit": tt["Sunlight ?"].to_numpy(dtype=float),
    }
    # Sunrise minute from the Parameters tab (row labelled "Sunrise").
    par = pd.read_excel(PWR00_XLSX, "Parameters", header=None)
    sunrise_min = None
    for _, row in par.iterrows():
        if isinstance(row.iloc[0], str) and row.iloc[0].strip().lower() == "sunrise":
            sunrise_min = float(row.iloc[1])
            break
    if sunrise_min is None:
        raise ValueError("could not find a 'Sunrise' row in the Parameters tab")
    return env, sunrise_min


def load_envelope():
    """
    (diss_min, diss_max) in W, keyed by absolute minute, from a previous
    Energy_Budget run's output CSV. Minutes outside the covered window
    clamp to its nearest edge. Returns (lookup_fn, have_envelope).
    """
    if not REFERENCE_CSV or not Path(REFERENCE_CSV).exists():
        print(f"  (no {REFERENCE_CSV} -- naps end on the battery criterion only)")
        return (lambda m, before_noon=True: (0.0, math.inf)), False

    df = pd.read_csv(REFERENCE_CSV)
    by_minute = {
        int(m): (mn, mx)
        for m, mn, mx in zip(df["Time [min]"], df["Dissipation min [W]"], df["Dissipation max [W]"])
        if 0 < mx < 1000  # 0 / 1000 are the "no data, don't use" sentinels
    }
    if not by_minute:
        print(f"  ({REFERENCE_CSV} has no usable dissipation rows -- heat term dropped)")
        return (lambda m, before_noon=True: (0.0, math.inf)), False
    lo, hi = min(by_minute), max(by_minute)

    def lookup(minute, before_noon=True):
        # Outside the reference run's covered window, clamp to its nearest
        # edge minute -- only the ~few hundred minutes of early egress fall
        # here, where the environment a few hours off is close enough.
        return by_minute.get(int(round(min(max(minute, lo), hi))), (0.0, math.inf))

    return lookup, True


# --------------------------------------------------------- rover timeline
def rover_task_stream(timeline, power_of):
    """
    Rover-actor tasks in start order, each as (name, task_id, power_w,
    duration_s). Zero-length tasks are dropped; ground (earth) tasks are
    left out -- they run in parallel and draw no rover power.
    """
    tasks = sorted((t for t in timeline if t["actor"] == "rover"), key=lambda t: t["start"])
    stream = []
    for t in tasks:
        dur = t["end"] - t["start"]
        if dur <= 0:
            continue
        stream.append({
            "name": t["name"], "task_id": t.get("task_id"),
            "power_w": power_of.get(t["name"], 0.0),
            "duration_s": dur, "orig_start": t["start"],
        })
    return stream


# --------------------------------------------------------- the simulation
def simulate(stream, env, envelope, have_env, env_offset_min, gen_start_min):
    e = EPS
    cap = e["eol_battery_capacity_wh"]
    hl = e["harness_losses_pct"]
    battery = cap
    stored_heat = 0.0

    # Precompute per-integer-minute arrays over the whole environment span
    # (the Timestep Timeline is already at 1-min resolution) so the loop is
    # array lookups, not repeated np.interp over a 42k-row table.
    em = env["minute"].astype(int)
    span = int(em.max()) + 1
    theta_a = np.full(span, np.nan)
    dist_a = np.full(span, np.nan)
    dust_a = np.zeros(span)
    sunlit_a = np.zeros(span, dtype=bool)
    theta_a[em] = env["sun_incidence_sa_deg"]
    dist_a[em] = env["sun_moon_dist_m"]
    dust_a[em] = env["dust_mg_cm2"]
    sunlit_a[em] = env["sunlit"] > 0.5
    noon_minute = int(em[int(np.nanargmin(env["sun_incidence_sa_deg"]))])

    with np.errstate(invalid="ignore"):
        raw_gen = e["solar_lum_eff_area"] / (4 * math.pi * dist_a ** 2) * np.cos(np.radians(theta_a))
    raw_gen = np.nan_to_num(np.clip(raw_gen, 0.0, None))
    rel = np.minimum(1.0, DUST_A * np.exp(DUST_B * dust_a))
    gen_a = (1 - hl / 100) * raw_gen * rel * e["pcdu_conversion_eff_pct"] / 100
    gen_a[~sunlit_a] = 0.0
    gen_a[: int(math.ceil(gen_start_min))] = 0.0

    if have_env:
        dmin_a = np.zeros(span)
        dmax_a = np.full(span, math.inf)
        for m in range(span):
            dmin_a[m], dmax_a[m] = envelope(m, before_noon=m <= noon_minute)
    else:
        dmin_a = np.zeros(span)
        dmax_a = np.full(span, math.inf)

    def step(consumption_w, abs_min):
        nonlocal battery, stored_heat
        idx = int(abs_min)
        gen = gen_a[idx] if idx < span else 0.0

        c_harness = consumption_w * (1 + hl / 100)
        c_pcdu = c_harness / (e["pcdu_distribution_eff_pct"] / 100)
        cq = 0.0 if c_pcdu == 0 else c_pcdu + e["pcdu_quiescent_w"] + e["battery_quiescent_w"]
        if cq <= gen:
            c_batt = cq
        else:
            c_batt = (cq - gen) / (e["battery_eff_pct"] / 100) + gen
        actual = c_batt * (1 + e["system_margin_pct"] / 100)

        d_min = dmin_a[idx] if idx < span else 0.0
        d_max = dmax_a[idx] if idx < span else math.inf
        heater = max(0.0, d_min - actual)
        excess = max(0.0, actual - d_max)
        spare = max(0.0, d_max - actual)
        stored_heat = max(0.0, stored_heat + excess / 60 - spare / 60)

        battery = min(cap, battery + (gen - (actual + heater)) / 60)
        return dict(minute=abs_min, generation=gen, dissipation=actual, heater=heater,
                    excess=excess, stored_heat=stored_heat, battery=battery,
                    consumption=actual + heater)

    log = []
    out_tasks = []
    t_min = 0.0          # mission clock, minutes since t=0
    carry = 0.0          # sub-minute remainder so total duration is preserved
    nap_lengths = []

    for task in stream:
        start_min = t_min
        is_nap = task["name"] in NAP_NAMES or task["task_id"] in NAP_IDS

        if is_nap:
            minutes = 0
            while True:
                rec = step(task["power_w"], env_offset_min + t_min)
                log.append({**rec, "activity": task["name"]})
                t_min += 1
                minutes += 1
                if minutes >= NAP_MAX_MIN:
                    break
                if minutes >= NAP_MIN_MIN and battery >= cap - 1e-9 and stored_heat <= 1e-9:
                    break
            nap_lengths.append(minutes)
            dur_min = minutes
        else:
            want = carry + task["duration_s"] / 60
            minutes = int(round(want))
            carry = want - minutes
            for _ in range(minutes):
                rec = step(task["power_w"], env_offset_min + t_min)
                log.append({**rec, "activity": task["name"]})
                t_min += 1
            dur_min = minutes

        out_tasks.append({
            "Activity": task["name"], "Activity ID": task["task_id"],
            "Start [min]": start_min, "Duration [min]": dur_min, "End [min]": start_min + dur_min,
            "Power [W]": task["power_w"], "is_nap": is_nap,
        })

    return out_tasks, pd.DataFrame(log), nap_lengths


# ----------------------------------------------------------------- main
def main():
    run_dir, config, results = load_run()
    print(f"Post-processing {run_dir}")
    mc = config.get("mission_config") or {}

    power_of = per_name_power(results["report"])
    stream = rover_task_stream(results["timeline"], power_of)

    planned_naps = [t for t in stream if t["name"] in NAP_NAMES or t["task_id"] in NAP_IDS]
    planned_nap_min = [t["duration_s"] / 60 for t in planned_naps]

    env, sunrise_min = load_environment()
    envelope, have_env = load_envelope()

    env_offset = ENV_OFFSET_MIN
    if env_offset is None:
        env_offset = sunrise_min + mc.get("mission_start_lunar_hour", 0.0) * 60
    print(f"  t=0 at Timestep-Timeline minute {env_offset:.0f} "
          f"(sunrise {sunrise_min:.0f} + {mc.get('mission_start_lunar_hour', 0.0)} lunar h)")

    gen_task = "Commissioning" if GENERATION_STARTS_AFTER == "commissioning" else "Egress"
    gen_start_rel = 0.0
    acc = 0.0
    for t in stream:
        acc += t["duration_s"] / 60
        if t["name"] == gen_task:
            gen_start_rel = acc
            break
    gen_start_min = env_offset + gen_start_rel
    print(f"  generation available from minute {gen_start_min:.0f} (after {gen_task})")

    out_tasks, log_df, nap_lengths = simulate(
        stream, env, envelope, have_env, env_offset, gen_start_min)

    orig_end = max(t["end"] for t in results["timeline"]) / 3600          # h
    new_end = out_tasks[-1]["End [min]"] / 60                             # h

    # ---- console summary ----
    def stats(xs):
        a = np.array(xs, dtype=float)
        return f"n={len(a)}  total={a.sum() / 60:.1f} h  mean={a.mean():.1f}  min={a.min():.0f}  max={a.max():.0f} min"

    print("\nNap durations (minutes):")
    print(f"  planned : {stats(planned_nap_min)}")
    print(f"  computed: {stats(nap_lengths)}")
    capped = sum(1 for n in nap_lengths if n >= NAP_MAX_MIN)
    floored = sum(1 for n in nap_lengths if n <= NAP_MIN_MIN)
    print(f"  at ceiling ({NAP_MAX_MIN} min): {capped}   at floor ({NAP_MIN_MIN} min): {floored}")
    print(f"\nMission end: {orig_end:.1f} h -> {new_end:.1f} h ({new_end - orig_end:+.1f} h)")
    print(f"Battery SoC: min {100 * log_df['battery'].min() / EPS['battery_capacity_wh']:.1f} %  "
          f"end {100 * log_df['battery'].iloc[-1] / EPS['battery_capacity_wh']:.1f} %")
    if have_env:
        print(f"Heater energy: {log_df['heater'].sum() / 60:.1f} Wh   "
              f"peak stored heat: {log_df['stored_heat'].max():.2f} Wh")

    # ---- write the corrected sequence ----
    seq_df = pd.DataFrame([{k: v for k, v in t.items() if k != "is_nap"} for t in out_tasks])
    out_path = run_dir / "Timeline_EnergyBudget.xlsx"
    with pd.ExcelWriter(out_path) as writer:
        seq_df.to_excel(writer, sheet_name="Timeline", index=False)
        cmp_df = pd.DataFrame({"planned_min": planned_nap_min, "computed_min": nap_lengths})
        cmp_df["delta_min"] = cmp_df["computed_min"] - cmp_df["planned_min"]
        cmp_df.to_excel(writer, sheet_name="Naps", index=False)
        log_df.to_excel(writer, sheet_name="Per-minute log", index=False)
    print(f"\nWrote {out_path}")

    # ---- plots ----
    nap_starts_h = [t["Start [min]"] / 60 for t in out_tasks if t["is_nap"]]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(nap_starts_h, planned_nap_min, ".", label="planned", color="#b0b0b0")
    axes[0].plot(nap_starts_h, nap_lengths, ".", label="computed", color="#2a78d6")
    axes[0].axhline(NAP_MAX_MIN, color="firebrick", lw=0.8, ls="--")
    axes[0].set_ylabel("Nap length [min]")
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"Energy-budget naps -- {run_dir.name}")

    log_h = log_df["minute"].to_numpy() / 60 - env_offset / 60
    axes[1].plot(log_h, 100 * log_df["battery"] / EPS["battery_capacity_wh"], color="cornflowerblue")
    axes[1].set_ylabel("Battery SoC [%]")
    axes[1].axhline(100, color="#b0b0b0", lw=0.8)

    axes[2].plot(log_h, log_df["dissipation"], color="darkorange", lw=0.7, label="dissipation")
    if have_env:
        axes[2].plot(log_h, log_df["heater"], color="red", lw=0.7, label="heater")
        axes[2].plot(log_h, log_df["stored_heat"], color="green", lw=0.7, label="stored heat [Wh]")
    axes[2].set_ylabel("Power [W]")
    axes[2].set_xlabel("Mission time [h]")
    axes[2].legend(loc="upper right")
    plt.tight_layout()
    fig.savefig(run_dir / "Timeline_EnergyBudget.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
