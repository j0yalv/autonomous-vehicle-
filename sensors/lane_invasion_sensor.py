import carla


class LaneInvasionSensor:

    def __init__(self, world, vehicle):

        blueprint_library = world.get_blueprint_library()

        lane_sensor_bp = blueprint_library.find(
            'sensor.other.lane_invasion'
        )

        self.sensor = world.spawn_actor(
            lane_sensor_bp,
            carla.Transform(),
            attach_to=vehicle
        )

        self.sensor.listen(self.on_invasion)

    def on_invasion(self, event):

        print("\nLANE INVASION DETECTED\n")

    def destroy(self):

        self.sensor.destroy()