from dataclasses import dataclass
import matplotlib.pyplot as plt



@dataclass
class Task:
    name: str
    actor: str
    start: float
    end: float

def generate_waypoint_tasks(start_time,
                            travel_time,
                            turn_time,
                            picture_time,
                            image_downlink_time,
                            relay_time,
                            earth_analysis_time,
                            measurement_time,
                            uplink_time,
                            nap_time,
                            config_power_robot):

    tasks = []

    t = start_time

    # Turn / reorientation (direction changes encountered while driving this leg)
    if turn_time > 0:
        tasks.append(Task("turn", "rover", t, t + turn_time))
        t += turn_time

    # Drive
    tasks.append(Task("drive", "rover", t, t + travel_time))
    t += travel_time

    # Picture
    tasks.append(Task("picture", "rover", t, t + picture_time))
    t += picture_time

    # Downlink picture to operator
    tasks.append(Task("image_downlink", "rover", t, t + image_downlink_time))
    t += image_downlink_time

    # Relay
    tasks.append(Task("relay", "rover", t, t + relay_time))
    relay_end = t + relay_time
    t += relay_time

    # Science Measurement
    tasks.append(Task("measurement", "rover", t, t + measurement_time))
    t += measurement_time

    # Nap
    '''
    average_power = config_power_robot["average_power"]
    
    total_time = t
    travel_time * config_power_robot["locomotion_power"]
    '''

    nap_time = nap_time
    tasks.append(Task("nap", "rover", t, t + nap_time))
    t += nap_time

    # Parallel activities
    tasks.append(Task("earth_analysis",
                      "earth",
                      relay_end,
                      relay_end + earth_analysis_time))

    # Uplink (must finish before next drive)
    uplink_start = max(relay_end + earth_analysis_time, measurement_time + relay_end + nap_time)

    tasks.append(Task("uplink",
                      "earth",
                      uplink_start,
                      uplink_start + uplink_time))

    return tasks, uplink_start + uplink_time

'''
def compute_nap_time(travel_time,config_time, config_power):
    average power
    total time 
    different times and powers
    travel_time* robot power, pictime*picpower,relay_time*relaypower, science_time*science power+const_uplink
    calculate power mean
    return nap_time
'''

def build_mission(segments, velocity, config_time, config_power_robot):
    """
    segments: list of {"distance": meters, "turns": count}, one per drive leg
    between measurement/downlink stops (see path_analyzer.build_drive_segments).
    """

    timeline = []
    t = 0

    for seg in segments:

        travel_time = seg["distance"] / velocity
        turn_time = seg["turns"] * config_time["turn_time"]

        tasks, t = generate_waypoint_tasks(
            t,
            travel_time,
            turn_time,
            config_time["picture_time"],
            config_time["image_downlink_time"],
            config_time["relay_time"],
            config_time["earth_analysis_time"],
            config_time["measurement_time"],
            config_time["uplink_time"],
            config_time["nap_time"],
            config_power_robot
        )

        timeline.extend(tasks)

    return timeline




def plot_timeline2(tasks):

    lanes = {
        #"rover": 10,
        #"earth": 20
        "drive": 0,
        "picture":1,
        "relay":2,
        "earth_analysis":3,
        "measurement":4
    }

    fig, ax = plt.subplots(figsize=(12,4))
    print (tasks)

    for task in tasks:
        ax.broken_barh(
            [(task.start, task.end-task.start)],
            (lanes[task.name], 5)
        )

    ax.set_yticks([0,5])
    ax.set_yticklabels(["rover", "earth"])
    ax.set_xlabel("Time")

    plt.show()

def plot_timeline(tasks, show=True, velocity=None):
    """
    velocity: rover drive speed [m/s]. If given, each "drive" bar is
    annotated with the distance covered in that leg (distance = velocity *
    duration) and the axes title shows the cumulative distance driven.
    """
    # collect unique task names
    names = set(task.name for task in tasks)
    names = names = list(dict.fromkeys(task.name for task in tasks))

    # map each name to a y position
    y_map = {name: i for i, name in enumerate(names)}

    fig, ax = plt.subplots(figsize=(14,5))

    cumulative_distance_m = 0.0
    for task in tasks:
        unit = 60 #transform from seconds to minutes
        start_time = task.start / 60
        duration_time = (task.end - task.start) / 60

        y = y_map[task.name]

        ax.broken_barh(
            [(start_time, duration_time)],
            (y - 0.4, 0.5)
        )

        if velocity is not None and task.name == "drive":
            leg_distance_m = velocity * (task.end - task.start)
            cumulative_distance_m += leg_distance_m
            ax.annotate(f"{leg_distance_m:.0f} m", (start_time + duration_time / 2, y),
                        ha="center", va="center", fontsize=8)

    ax.set_yticks(list(y_map.values()))
    ax.set_yticklabels(list(y_map.keys()))
    ax.set_xlabel("Time [min]")
    if velocity is not None:
        ax.set_title(f"Total distance driven: {cumulative_distance_m:.0f} m")

    plt.tight_layout()
    if show:
        plt.show()

    return fig



