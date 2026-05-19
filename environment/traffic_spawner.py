import random

import carla

from environment.actor_cleanup import ActorCleanup


class TrafficSpawner:

    def __init__(
        self,
        client,
        world,
        ego_vehicle,
        vehicle_count=28,
        pedestrian_count=12
    ):

        self.client = client
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.vehicle_count = vehicle_count
        self.pedestrian_count = pedestrian_count
        self.vehicles = []
        self.walkers = []
        self.walker_controllers = []

    def spawn(self):

        self.spawn_vehicles()
        self.spawn_pedestrians()

        print(
            f"Spawned {len(self.vehicles)} NPC vehicles and "
            f"{len(self.walkers)} pedestrians"
        )

    def spawn_vehicles(self):

        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter('vehicle.*')
        spawn_points = self.world.get_map().get_spawn_points()
        ego_location = self.ego_vehicle.get_location()

        random.shuffle(spawn_points)

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)

        for spawn_point in spawn_points:

            if len(self.vehicles) >= self.vehicle_count:
                break

            if spawn_point.location.distance(ego_location) < 12.0:
                continue

            blueprint = random.choice(vehicle_blueprints)

            if blueprint.has_attribute('color'):
                color = random.choice(
                    blueprint.get_attribute('color').recommended_values
                )
                blueprint.set_attribute('color', color)

            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'autonomous_npc')

            npc_vehicle = self.world.try_spawn_actor(
                blueprint,
                spawn_point
            )

            if npc_vehicle is None:
                continue

            npc_vehicle.set_autopilot(
                True,
                traffic_manager.get_port()
            )

            self.vehicles.append(npc_vehicle)

    def spawn_pedestrians(self):

        blueprint_library = self.world.get_blueprint_library()
        walker_blueprints = blueprint_library.filter('walker.pedestrian.*')
        controller_bp = blueprint_library.find('controller.ai.walker')

        for _ in range(self.pedestrian_count):

            location = self.world.get_random_location_from_navigation()

            if location is None:
                continue

            blueprint = random.choice(walker_blueprints)

            if blueprint.has_attribute('is_invincible'):
                blueprint.set_attribute('is_invincible', 'false')

            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'autonomous_pedestrian')

            walker = self.world.try_spawn_actor(
                blueprint,
                carla.Transform(location)
            )

            if walker is None:
                continue

            controller = self.world.try_spawn_actor(
                controller_bp,
                carla.Transform(),
                walker
            )

            if controller is None:
                walker.destroy()
                continue

            controller.start()

            target_location = self.world.get_random_location_from_navigation()

            if target_location is not None:
                controller.go_to_location(target_location)

            controller.set_max_speed(random.uniform(1.0, 1.8))

            self.walkers.append(walker)
            self.walker_controllers.append(controller)

    def destroy(self):

        print("Cleaning up spawned traffic actors")

        destroyed_controllers = ActorCleanup.destroy_actors(
            self.walker_controllers
        )

        destroyed_walkers = ActorCleanup.destroy_actors(self.walkers)

        destroyed_vehicles = ActorCleanup.destroy_actors(self.vehicles)

        print(
            f"Traffic cleanup destroyed "
            f"{destroyed_vehicles} vehicles, "
            f"{destroyed_walkers} pedestrians, "
            f"{destroyed_controllers} walker controllers"
        )

        self.walker_controllers = []
        self.walkers = []
        self.vehicles = []
