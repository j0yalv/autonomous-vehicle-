import math

import carla


class ObstacleAwareness:

    def __init__(
        self,
        world,
        vehicle,
        detection_distance=18.0,
        cautious_distance=12.0,
        brake_distance=6.0,
        lane_width=3.5
    ):

        self.world = world
        self.vehicle = vehicle
        self.detection_distance = detection_distance
        self.cautious_distance = cautious_distance
        self.brake_distance = brake_distance
        self.lane_width = lane_width

    def get_obstacle_ahead(self):

        ego_transform = self.vehicle.get_transform()
        ego_location = ego_transform.location
        forward_vector = ego_transform.get_forward_vector()

        closest_actor = None
        closest_distance = self.detection_distance

        actors = list(
            self.world.get_actors().filter('vehicle.*')
        ) + list(
            self.world.get_actors().filter('walker.pedestrian.*')
        )

        for actor in actors:

            if actor.id == self.vehicle.id:
                continue

            actor_location = actor.get_location()

            offset_x = actor_location.x - ego_location.x
            offset_y = actor_location.y - ego_location.y

            forward_distance = (
                offset_x * forward_vector.x +
                offset_y * forward_vector.y
            )

            if (
                forward_distance <= 0.0 or
                forward_distance > self.detection_distance
            ):
                continue

            total_distance = math.sqrt(
                offset_x ** 2 +
                offset_y ** 2
            )

            lateral_distance = math.sqrt(
                max(total_distance ** 2 - forward_distance ** 2, 0.0)
            )

            if lateral_distance > self.lane_width:
                continue

            if forward_distance < closest_distance:
                closest_distance = forward_distance
                closest_actor = actor

        if closest_actor is None:
            return None, None

        return closest_actor, closest_distance

    def apply_reactive_speed(self, target_speed):

        obstacle, distance = self.get_obstacle_ahead()

        if obstacle is None:
            return target_speed, False, None, None

        self.world.debug.draw_line(
            self.vehicle.get_location(),
            obstacle.get_location(),
            thickness=0.1,
            color=carla.Color(255, 165, 0),
            life_time=0.1
        )

        if distance <= self.brake_distance:
            return 0.0, True, obstacle, distance

        if distance <= self.cautious_distance:
            return target_speed * 0.35, False, obstacle, distance

        return target_speed * 0.65, False, obstacle, distance
