# Generated from user_data/config_robot/Input_Conops.xlsx (sheet "Robot Time & Power") by
# import_conops_excel.py -- re-run that script after updating the Excel;
# don't hand-edit the commented values below, edit the spreadsheet instead.
# All values in Watt.
config = {
    "drive_power": 159.91,  # "Walking + GPR meas. duty cycled" (OPS-03)
    "turn_power": 151.51,  # "Spot Turn"
    "picture_power": 17.11,  # "NAV image capture" (OPS-10)
    "image_downlink_power": 15.41,  # "NAV downlink+ GRAV Spinup" (OPS-12-a)
    "grav_measurement_power": 15.01,  # "GRAV measurement" (OPS-13)
    "spec_measurement_power": 11.81,  # "SPEC measurement" (OPS-15)
    "scicam_measurement_power": 13.01,  # "SciCAM measurement" (OPS-17)
    "relay_power": 15.42,  # "Payload data downlink" (OPS-12-b)
    "nap_power": 5.0,  # "Nap" (OPS-20)
    "earth_analysis_power": 0.0,  # "Mission Ops Decision"
    "uplink_power": 0.0,  # "Uplink"
}
