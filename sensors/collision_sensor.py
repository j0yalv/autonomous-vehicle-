import carla

class CollisionSensor:

    def __init__(self, world, vehicle):

        blueprint_library = world.get_blueprint_library()

        collision_bp = blueprint_library.find('sensor.other.collision')

        self.sensor = world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=vehicle
        )

        self.sensor.listen(self.on_collision)

    def on_collision(self, event):

        actor = event.other_actor

        print(f"\nCOLLISION WITH: {actor.type_id}\n")

    def destroy(self):

        self.sensor.destroy()