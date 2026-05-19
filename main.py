import carla
import time
import math

from controllers.pid_controller import PIDController

from sensors.collision_sensor import CollisionSensor
from sensors.lane_invasion_sensor import LaneInvasionSensor
from sensors.camera_manager import CameraManager

from environment.actor_cleanup import ActorCleanup
from environment.obstacle_awareness import ObstacleAwareness
from environment.speed_smoother import SpeedSmoother
from environment.traffic_light_handler import TrafficLightHandler
from environment.traffic_spawner import TrafficSpawner

from utils.metrics import Metrics
from utils.plot_metrics import PlotMetrics


def safe_destroy(name, obj):

    if obj is None:
        return

    try:
        obj.destroy()
        print(f"{name} destroyed")

    except Exception as exc:
        print(f"{name} cleanup skipped: {exc}")


# ---------------- CARLA Setup ---------------- #

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()

map = world.get_map()

blueprint_library = world.get_blueprint_library()

vehicle = None
collision_sensor = None
lane_sensor = None
camera_manager = None
traffic_spawner = None
plot_metrics = None

try:

    ActorCleanup.cleanup_existing_actors(world)

    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    if vehicle_bp.has_attribute('role_name'):
        vehicle_bp.set_attribute('role_name', 'autonomous_ego')

    spawn_points = map.get_spawn_points()

    spawn_point = spawn_points[25]

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    print(f"Vehicle spawned: {vehicle.id}")

    
    # ---------------- Sensors ---------------- #

    collision_sensor = CollisionSensor(world, vehicle)

    lane_sensor = LaneInvasionSensor(world, vehicle)

    camera_manager = CameraManager(world, vehicle)


    # ---------------- Environment Awareness ---------------- #

    traffic_spawner = TrafficSpawner(
        client,
        world,
        vehicle
    )

    traffic_spawner.spawn()

    obstacle_awareness = ObstacleAwareness(
        world,
        vehicle
    )

    traffic_light_handler = TrafficLightHandler(
        world,
        vehicle
    )

    speed_smoother = SpeedSmoother()


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

        cruise_speed = 5.0
        target_speed = cruise_speed

        (
            target_speed,
            emergency_brake,
            obstacle,
            obstacle_distance
        ) = obstacle_awareness.apply_reactive_speed(
            target_speed,
            current_speed
        )

        (
            target_speed,
            traffic_light_brake,
            traffic_light_state
        ) = traffic_light_handler.apply_reactive_speed(target_speed)

        emergency_brake = emergency_brake or traffic_light_brake

        desired_target_speed = target_speed

        target_speed = speed_smoother.smooth_target_speed(
            desired_target_speed,
            dt,
            emergency_brake
        )

        speed_error = target_speed - current_speed

        # -------- Speed PID -------- #

        throttle_output = speed_pid.control(speed_error, dt)

        throttle = 0.0
        brake = 0.0

        if throttle_output >= 0:
            throttle = min(throttle_output, 1.0)
        else:
            brake = min(abs(throttle_output), 1.0)

        if emergency_brake:
            throttle = 0.0
            brake = 1.0

        throttle, brake = speed_smoother.smooth_control(
            throttle,
            brake,
            dt,
            emergency_brake
        )

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

        obstacle_message = ""

        if obstacle is not None:
            obstacle_message = (
                f" | Obstacle: {obstacle.type_id} "
                f"{obstacle_distance:.1f}m"
            )

        traffic_light_message = ""

        if traffic_light_state is not None:
            traffic_light_message = (
                f" | Light: {traffic_light_state}"
            )

        print(
            f"Step {i} | "
            f"Speed: {current_speed:.2f} | "
            f"Desired: {desired_target_speed:.2f} | "
            f"Target: {target_speed:.2f} | "
            f"Steer: {steer:.2f}"
            f"{obstacle_message}"
            f"{traffic_light_message}"
        )

        time.sleep(dt)


    # ---------------- Stop Vehicle ---------------- #

    if vehicle is not None and vehicle.is_alive:
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

finally:

    # ---------------- Cleanup ---------------- #

    if vehicle is not None and vehicle.is_alive:
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0
            )
        )

    safe_destroy("Collision sensor", collision_sensor)

    safe_destroy("Lane invasion sensor", lane_sensor)

    safe_destroy("Camera manager", camera_manager)

    safe_destroy("Traffic spawner", traffic_spawner)

    ActorCleanup.destroy_actor(vehicle)

    # ---------------- Close OpenCV ---------------- #

    try:
        import cv2

        cv2.destroyAllWindows()

    except Exception as exc:
        print(f"OpenCV cleanup skipped: {exc}")

    time.sleep(1)


# ---------------- Plot Graphs ---------------- #

if plot_metrics is not None:
    plot_metrics.plot()
