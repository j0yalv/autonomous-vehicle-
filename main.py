import carla
import time
import math
import random

from controllers.pid_controller import PIDController

from sensors.collision_sensor import CollisionSensor
from sensors.lane_invasion_sensor import LaneInvasionSensor
# from sensors.camera_manager import CameraManager

from utils.metrics import Metrics
from utils.plot_metrics import PlotMetrics


# ---------------- CARLA Setup ---------------- #

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()

map = world.get_map()

blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

spawn_points = map.get_spawn_points()

spawn_point = spawn_points[20]

vehicle = world.spawn_actor(vehicle_bp, spawn_point)

print("Vehicle spawned")


# ---------------- Sensors ---------------- #

collision_sensor = CollisionSensor(world, vehicle)

lane_sensor = LaneInvasionSensor(world, vehicle)

# camera_manager = CameraManager(world, vehicle)


# ---------------- Metrics ---------------- #

metrics = Metrics()

plot_metrics = PlotMetrics()


# ---------------- PID Setup ---------------- #

steering_pid = PIDController(
    kp=0.5,
    ki=0.0,
    kd=0.08
)

speed_pid = PIDController(
    kp=0.4,
    ki=0.0,
    kd=0.05
)


# ---------------- Main Loop ---------------- #

dt = 0.05

for i in range(1000):

    transform = vehicle.get_transform()

    vehicle_location = vehicle.get_location()

    # -------- Spectator Camera -------- #

    spectator = world.get_spectator()

    forward_vector = transform.get_forward_vector()

    camera_location = carla.Location(
        x=transform.location.x - forward_vector.x * 10,
        y=transform.location.y - forward_vector.y * 10,
        z=transform.location.z + 5
    )

    camera_rotation = carla.Rotation(
        pitch=-15,
        yaw=transform.rotation.yaw,
        roll=0
    )

    spectator.set_transform(
        carla.Transform(
            camera_location,
            camera_rotation
        )
    )

    # -------- Get Closest Lane Waypoint -------- #

    current_waypoint = map.get_waypoint(
        vehicle_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    # -------- Look Ahead Waypoint -------- #

    next_waypoints = current_waypoint.next(5.0)

    if len(next_waypoints) == 0:
        break

    target_waypoint = next_waypoints[0]

    target_location = target_waypoint.transform.location

    # -------- Debug Visualization -------- #

    world.debug.draw_point(
        target_location,
        size=0.15,
        color=carla.Color(255, 0, 0),
        life_time=0.1
    )

    # -------- Vehicle Orientation -------- #

    vehicle_yaw = math.radians(transform.rotation.yaw)

    # -------- Direction To Waypoint -------- #

    direction_x = target_location.x - vehicle_location.x
    direction_y = target_location.y - vehicle_location.y

    desired_yaw = math.atan2(direction_y, direction_x)

    # -------- Heading Error -------- #

    heading_error = desired_yaw - vehicle_yaw

    while heading_error > math.pi:
        heading_error -= 2 * math.pi

    while heading_error < -math.pi:
        heading_error += 2 * math.pi

    # -------- Steering PID -------- #

    steer = steering_pid.control(heading_error, dt)

    steer = max(-1.0, min(1.0, steer))

    # -------- Current Speed -------- #

    velocity = vehicle.get_velocity()

    current_speed = math.sqrt(
        velocity.x ** 2 +
        velocity.y ** 2 +
        velocity.z ** 2
    )

    # -------- Target Speed -------- #

    target_speed = 5.0

    speed_error = target_speed - current_speed

    # -------- Speed PID -------- #

    throttle_output = speed_pid.control(speed_error, dt)

    throttle = 0.0
    brake = 0.0

    if throttle_output >= 0:
        throttle = min(throttle_output, 1.0)
    else:
        brake = min(abs(throttle_output), 1.0)

    # -------- Vehicle Control -------- #

    control = carla.VehicleControl(
        throttle=throttle,
        steer=steer,
        brake=brake
    )

    vehicle.apply_control(control)

    # -------- Metrics Recording -------- #

    metrics.record(
        current_speed,
        heading_error,
        steer
    )

    plot_metrics.record(
        current_speed,
        heading_error,
        steer
    )

    print(
        f"Step {i} | "
        f"Speed: {current_speed:.2f} | "
        f"Steer: {steer:.2f}"
    )

    time.sleep(dt)


# ---------------- Stop Vehicle ---------------- #

vehicle.apply_control(
    carla.VehicleControl(
        throttle=0.0,
        steer=0.0,
        brake=1.0
    )
)

time.sleep(2)


# ---------------- Metrics Summary ---------------- #

metrics.summary()


# ---------------- Cleanup ---------------- #

collision_sensor.destroy()

lane_sensor.destroy()

# camera_manager.destroy()

vehicle.destroy()

print("Vehicle destroyed")


# ---------------- Close OpenCV ---------------- #

import cv2

cv2.destroyAllWindows()

time.sleep(1)


# ---------------- Plot Graphs ---------------- #

plot_metrics.plot()