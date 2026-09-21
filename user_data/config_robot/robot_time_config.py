# Generated from user_data/config_robot/Input_Conops.xlsx (sheet "Robot Time & Power") by
# import_conops_excel.py -- re-run that script after updating the Excel;
# don't hand-edit the commented values below, edit the spreadsheet instead.
# All values in seconds.
config = {
    "turn_time": 30,  # "Spot Turn" (OPS-23)
    "standing_time": 30,  # "Standing in any pose (braking)" (OPS-07)
    "picture_time": 5,  # "NAV image capture" (OPS-10)
    "image_downlink_time": 30,  # "NAV downlink+ GRAV Spinup" (OPS-12-a)
    "grav_measurement_time": 180,  # "GRAV measurement" (OPS-13)
    "spec_measurement_time": 120,  # "SPEC measurement" (OPS-15)
    "spec_warmup_time": 1800,  # "SPEC warmup" (OPS-14)
    "scicam_measurement_time": 5,  # "SciCAM measurement" (OPS-17)
    "relay_time": 342.85714285714283,  # "Payload data downlink" (OPS-12-b)
    "nap_time": 400,  # "Nap" (OPS-20)
    "earth_analysis_time": 300,  # "Mission Ops Decision"
    "uplink_time": 15,  # "Uplink"
}
