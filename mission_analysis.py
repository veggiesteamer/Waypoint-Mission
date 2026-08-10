import matplotlib.pyplot as plt
import numpy as np

class Task:
    def __init__(self, name, start, duration, category):
        self.name = name
        self.start = start
        self.duration = duration
        self.end = start + duration
        self.category = category

def travel_task(tasks, start_time, distance, velocity):

    duration = distance / velocity

    tasks.append(Task(
        "Drive",
        start_time,
        duration,
        "rover_motion"
    ))

    return start_time + duration

def picture_task(tasks, start_time, duration):

    tasks.append(Task(
        "Take Picture",
        start_time,
        duration,
        "rover_camera"
    ))

    return start_time + duration

def relay_task(tasks, start_time, relay_time, analysis_time):

    tasks.append(Task(
        "Relay Data",
        start_time,
        relay_time,
        "relay"
    ))

    tasks.append(Task(
        "Earth Analysis",
        start_time,
        analysis_time,
        "earth"
    ))

    return start_time + relay_time

def measurement_campaign(tasks, start_time, duration):

    tasks.append(Task(
        "Surface Measurement",
        start_time,
        duration,
        "science"
    ))

    return start_time + duration

def build_mission(segments, velocity):

    tasks = []
    time = 0

    for d in segments:

        time = travel_task(tasks, time, d, velocity)

        time = picture_task(tasks, time, 5)

        time = relay_task(tasks, time, 120, 15*60)

        time = measurement_campaign(tasks, time, 240)

    return tasks


CATEGORY_Y = {
    "rover_motion":0,
    "rover_camera":1,
    "relay":2,
    "science":3,
    "earth":4
}


tasks = build_mission(segments, robot_velocity)
fig, ax = plt.subplots(figsize=(14,5))

for task in tasks:

    y = CATEGORY_Y[task.category]

    ax.broken_barh(
        [(task.start, task.duration)],
        (y-0.4, 0.8)
    )

ax.set_yticks(list(CATEGORY_Y.values()))
ax.set_yticklabels(list(CATEGORY_Y.keys()))

ax.set_xlabel("Time")
ax.set_ylabel("Activity")

plt.show()