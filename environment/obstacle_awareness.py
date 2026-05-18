import math

import carla


class ObstacleAwareness:

    def __init__(
        self,
        world,
        vehicle,
        detection_distance=40.0,
        cautious_distance=18.0,
        brake_distance=7.0,
        lane_width=3.5,
        min_following_distance=12.0,
        time_headway=2.2
    ):

        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        self.detection_distance = detection_distance
        self.cautious_distance = cautious_distance
        self.brake_distance = brake_distance
        self.lane_width = lane_width
        self.min_following_distance = min_following_distance
        self.time_headway = time_headway

    def get_obstacle_ahead(self):

        ego_transform = self.vehicle.get_transform()
        ego_location = ego_transform.location
        forward_vector = ego_transform.get_forward_vector()
        ego_waypoint = self._get_driving_waypoint(ego_location)

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

            if actor.type_id.startswith('vehicle.'):
                actor_waypoint = self._get_driving_waypoint(actor_location)

                blocking, reason = self._is_vehicle_blocking_path(
                    ego_waypoint,
                    actor_waypoint,
                    lateral_distance
                )

                self._log_vehicle_filter(
                    actor,
                    forward_distance,
                    lateral_distance,
                    ego_waypoint,
                    actor_waypoint,
                    reason
                )

                if not blocking:
                    continue

            elif lateral_distance > self.lane_width:
                continue

            if forward_distance < closest_distance:
                closest_distance = forward_distance
                closest_actor = actor

        if closest_actor is None:
            return None, None

        return closest_actor, closest_distance

    def apply_reactive_speed(self, target_speed, current_speed=0.0):

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

        if obstacle.type_id.startswith('vehicle.'):
            return self._follow_vehicle(
                obstacle,
                distance,
                target_speed,
                current_speed
            )

        if distance <= self.brake_distance:
            return 0.0, True, obstacle, distance

        if distance <= self.cautious_distance:
            return target_speed * 0.35, False, obstacle, distance

        return target_speed * 0.65, False, obstacle, distance

    def _follow_vehicle(
        self,
        obstacle,
        distance,
        target_speed,
        current_speed
    ):

        lead_speed = self._get_actor_speed(obstacle)
        closing_speed = max(current_speed - lead_speed, 0.0)

        safe_distance = max(
            self.min_following_distance,
            current_speed * self.time_headway
        )

        danger_distance = max(
            self.brake_distance,
            current_speed * 1.0
        )

        approach_distance = safe_distance + max(10.0, current_speed * 1.8)
        time_to_collision = None

        if closing_speed > 0.1:
            time_to_collision = distance / closing_speed

        if (
            distance <= danger_distance or
            (
                time_to_collision is not None and
                time_to_collision < 1.8
            ) or
            (distance < safe_distance * 0.65 and closing_speed > 0.5)
        ):
            print(
                f"Following vehicle {obstacle.id} | "
                f"distance: {distance:.1f}m | "
                f"safe: {safe_distance:.1f}m | "
                f"danger: {danger_distance:.1f}m | "
                f"ego: {current_speed:.2f}m/s | "
                f"lead: {lead_speed:.2f}m/s | "
                f"closing: {closing_speed:.2f}m/s | "
                "decision: emergency brake"
            )

            return 0.0, True, obstacle, distance

        if distance < safe_distance:
            gap_ratio = max(distance / safe_distance, 0.2)
            follow_speed = lead_speed * gap_ratio
            follow_speed = min(target_speed, follow_speed)

            print(
                f"Following vehicle {obstacle.id} | "
                f"distance: {distance:.1f}m | "
                f"safe: {safe_distance:.1f}m | "
                f"danger: {danger_distance:.1f}m | "
                f"ego: {current_speed:.2f}m/s | "
                f"lead: {lead_speed:.2f}m/s | "
                f"closing: {closing_speed:.2f}m/s | "
                f"target: {follow_speed:.2f}m/s | "
                "decision: proportional braking"
            )

            return (
                follow_speed,
                False,
                obstacle,
                distance
            )

        if distance < approach_distance or closing_speed > 0.3:
            approach_ratio = (
                (distance - safe_distance) /
                max(approach_distance - safe_distance, 0.1)
            )
            approach_ratio = max(0.0, min(1.0, approach_ratio))

            closing_penalty = min(closing_speed * 0.8, 2.5)
            matched_speed = lead_speed + approach_ratio * 2.0
            follow_speed = max(matched_speed - closing_penalty, 0.0)
            follow_speed = min(target_speed, follow_speed)

            print(
                f"Following vehicle {obstacle.id} | "
                f"distance: {distance:.1f}m | "
                f"safe: {safe_distance:.1f}m | "
                f"approach: {approach_distance:.1f}m | "
                f"ego: {current_speed:.2f}m/s | "
                f"lead: {lead_speed:.2f}m/s | "
                f"closing: {closing_speed:.2f}m/s | "
                f"target: {follow_speed:.2f}m/s | "
                "decision: early speed reduction"
            )

            return (
                follow_speed,
                False,
                obstacle,
                distance
            )

        print(
            f"Following vehicle {obstacle.id} | "
            f"distance: {distance:.1f}m | "
            f"safe: {safe_distance:.1f}m | "
            f"approach: {approach_distance:.1f}m | "
            f"ego: {current_speed:.2f}m/s | "
            f"lead: {lead_speed:.2f}m/s | "
            f"target: {target_speed:.2f}m/s | "
            "decision: hold cruise"
        )

        return target_speed, False, obstacle, distance

    def _get_actor_speed(self, actor):

        velocity = actor.get_velocity()

        return math.sqrt(
            velocity.x ** 2 +
            velocity.y ** 2 +
            velocity.z ** 2
        )

    def _get_driving_waypoint(self, location):

        return self.map.get_waypoint(
            location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

    def _is_vehicle_blocking_path(
        self,
        ego_waypoint,
        actor_waypoint,
        lateral_distance
    ):

        tight_path_distance = self.lane_width * 0.45

        if ego_waypoint is None or actor_waypoint is None:
            if lateral_distance <= tight_path_distance:
                return True, "blocking: waypoint unavailable but in path"

            return False, "ignored: waypoint unavailable and off path"

        same_lane = (
            ego_waypoint.road_id == actor_waypoint.road_id and
            ego_waypoint.lane_id == actor_waypoint.lane_id
        )

        if same_lane:
            return True, "blocking: same road/lane"

        if ego_waypoint.is_junction or actor_waypoint.is_junction:
            if lateral_distance <= tight_path_distance:
                return True, "blocking: junction and in path"

            return False, "ignored: junction but outside path"

        return False, "ignored: adjacent or different lane"

    def _log_vehicle_filter(
        self,
        actor,
        forward_distance,
        lateral_distance,
        ego_waypoint,
        actor_waypoint,
        reason
    ):

        ego_lane = "unknown"
        actor_lane = "unknown"

        if ego_waypoint is not None:
            ego_lane = f"{ego_waypoint.road_id}:{ego_waypoint.lane_id}"

        if actor_waypoint is not None:
            actor_lane = f"{actor_waypoint.road_id}:{actor_waypoint.lane_id}"

        print(
            f"Vehicle filter {actor.id} | "
            f"forward: {forward_distance:.1f}m | "
            f"lateral: {lateral_distance:.1f}m | "
            f"ego lane: {ego_lane} | "
            f"actor lane: {actor_lane} | "
            f"{reason}"
        )
