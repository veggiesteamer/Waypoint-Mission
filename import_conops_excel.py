"""
One-off importer: reads user_data/config_robot/Input_Conops.xlsx (sheet
"Robot Time & Power" only -- "Tabelle1" is a separate power-budget lookup
table, not the mission sequence) and regenerates robot_time_config.py /
robot_power_config.py as clean, hand-editable Python dicts.

Re-run this after updating the Excel. Mission planning itself never reads
the spreadsheet directly, only these two generated files -- so a formatting
quirk in the sheet (it has a few: a mis-encoded degree sign, missing power
cells) can only break this import step, never a planning run.
"""
from pathlib import Path

import openpyxl

EXCEL_PATH = Path("user_data/config_robot/Input_Conops.xlsx")
SHEET_NAME = "Robot Time & Power"
TIME_CONFIG_PATH = Path("user_data/config_robot/robot_time_config.py")
POWER_CONFIG_PATH = Path("user_data/config_robot/robot_power_config.py")

ACTOR_BY_SHEET_VALUE = {"Robot": "rover", "Ground": "earth"}

# Task ID -> (internal key, display name shown on the timeline, actor).
# Preferred over description-text matching since it's immune to the sheet's
# text quirks (e.g. the "SciCAM 360 measurement" row uses a mis-encoded
# masculine-ordinal-indicator, 0xBA, instead of a real degree sign, 0xB0).
ROW_MAP_BY_ID = {
    "OPS-03": ("drive", "drive", "rover"),
    "OPS-22": ("scicam_360", "SciCAM 360 measurement", "rover"),
    "OPS-10": ("picture", "Situational Picture", "rover"),
    "OPS-12-a": ("image_downlink", "Nav Downlink", "rover"),
    "OPS-13": ("grav_measurement", "GRAV measurement", "rover"),
    "OPS-15": ("spec_measurement", "SPEC measurement", "rover"),
    "OPS-17": ("scicam_measurement", "SciCAM measurement", "rover"),
    "OPS-20": ("nap", "nap", "rover"),
    # takes the old "relay" slot (second Nav Downlink, right after the
    # first) -- the sheet replaced that row with this one
    "OPS-12-b": ("relay", "Payload Downlink", "rover"),
}

# Fallback for the rows the sheet gives no Task ID for -- matched on the
# (plain-ASCII) description text instead.
ROW_MAP_BY_DESCRIPTION = {
    "Spot Turn": ("turn", "turn", "rover"),
    "Mission Ops Decision": ("earth_analysis", "earth_analysis", "earth"),
    "Uplink": ("uplink", "uplink", "earth"),
}


def load_rows():
    wb = openpyxl.load_workbook(EXCEL_PATH, data_only=True)
    ws = wb[SHEET_NAME]
    rows = list(ws.iter_rows(values_only=True))
    _header, *data_rows = rows

    entries = []
    for row in data_rows:
        description, actor_cell, _phase, task_id, power, time_s = row
        if description is None:
            continue
        description = description.strip()

        if task_id in ROW_MAP_BY_ID:
            key, display, actor = ROW_MAP_BY_ID[task_id]
        elif description in ROW_MAP_BY_DESCRIPTION:
            key, display, actor = ROW_MAP_BY_DESCRIPTION[description]
        else:
            raise ValueError(
                f"Unmapped row: description={description!r}, Task ID={task_id!r} in "
                f"{EXCEL_PATH}::{SHEET_NAME} -- add it to ROW_MAP_BY_ID/ROW_MAP_BY_DESCRIPTION "
                f"in import_conops_excel.py")

        expected_actor = ACTOR_BY_SHEET_VALUE.get(actor_cell)
        if expected_actor != actor:
            print(f"WARNING: {description!r} (Task ID {task_id!r}) has Actor={actor_cell!r} "
                  f"in the sheet but is mapped to actor={actor!r} -- check ROW_MAP")

        if power is None:
            print(f"WARNING: no power value for {description!r} (Task ID {task_id!r}) "
                  f"-- defaulting to 0 W, fill in by hand once known")
            power = 0.0

        if key == "drive":
            if time_s != "Distance":
                print(f"WARNING: expected the drive row's Time to read \"Distance\", got "
                      f"{time_s!r} -- drive duration always comes from the planned route "
                      f"and this value is ignored")
            time_s = None
        elif not isinstance(time_s, (int, float)):
            raise ValueError(f"Non-numeric Time for {description!r}: {time_s!r}")

        entries.append({"key": key, "display": display, "actor": actor,
                         "description": description, "task_id": task_id,
                         "power_w": float(power), "time_s": time_s})

    seen_keys = {e["key"] for e in entries}
    expected_keys = {v[0] for v in ROW_MAP_BY_ID.values()} | {v[0] for v in ROW_MAP_BY_DESCRIPTION.values()}
    missing = expected_keys - seen_keys
    if missing:
        print(f"WARNING: expected tasks not found in the sheet: {sorted(missing)}")

    return entries


def _format_config(entries, value_field, suffix, extra_lines, header_note):
    lines = [
        f'# Generated from {EXCEL_PATH.as_posix()} (sheet "{SHEET_NAME}") by',
        "# import_conops_excel.py -- re-run that script after updating the Excel;",
        "# don't hand-edit the commented values below, edit the spreadsheet instead.",
        f"# {header_note}" if header_note else None,
        "config = {",
    ]
    lines = [l for l in lines if l is not None]
    for e in entries:
        if e[value_field] is None:
            continue
        # the sheet uses a mis-encoded masculine-ordinal-indicator (0xBA)
        # instead of a real degree sign (0xB0) in "SciCAM 360 measurement" --
        # normalize it here so the generated file is plain, valid UTF-8
        description = e["description"].replace("\xba", "\xb0")
        source = f'"{description}"' + (f" ({e['task_id']})" if e["task_id"] else "")
        lines.append(f'    "{e["key"]}_{suffix}": {e[value_field]!r},  # {source}')
    lines.extend(extra_lines)
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def main():
    entries = load_rows()

    # "localization turn" (see mission_ops_config.localization_interval_
    # measurements) has no config entry of its own -- it's the same
    # physical maneuver as SciCAM 360 measurement, so conops_planner reuses
    # scicam_360_time/scicam_360_power directly.

    TIME_CONFIG_PATH.write_text(_format_config(
        entries, "time_s", "time", [], "All values in seconds."), encoding="utf-8")
    POWER_CONFIG_PATH.write_text(_format_config(
        entries, "power_w", "power", [], "All values in Watt."), encoding="utf-8")

    print(f"Wrote {TIME_CONFIG_PATH} and {POWER_CONFIG_PATH}")


if __name__ == "__main__":
    main()
