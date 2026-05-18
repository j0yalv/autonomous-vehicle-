import carla


class TrafficLightHandler:

    def __init__(self, world, vehicle):

        self.world = world
        self.vehicle = vehicle
        self.last_light_id = None
        self.last_state = None

    def apply_reactive_speed(self, target_speed):

        if not self.vehicle.is_at_traffic_light():
            self._print_state_change(None, None)
            return target_speed, False, None

        traffic_light = self.vehicle.get_traffic_light()

        if traffic_light is None:
            self._print_state_change(None, None)
            return target_speed, False, None

        state = self.vehicle.get_traffic_light_state()
        self._print_state_change(traffic_light.id, state)

        self.world.debug.draw_line(
            self.vehicle.get_location(),
            traffic_light.get_location(),
            thickness=0.12,
            color=self._state_color(state),
            life_time=0.1
        )

        if state == carla.TrafficLightState.Red:
            return 0.0, True, state

        return target_speed, False, state

    def _print_state_change(self, light_id, state):

        if light_id == self.last_light_id and state == self.last_state:
            return

        self.last_light_id = light_id
        self.last_state = state

        if light_id is None:
            print("Traffic light: none")
            return

        print(
            f"Traffic light {light_id}: "
            f"{self._state_name(state)}"
        )

    def _state_name(self, state):

        return str(state).replace("TrafficLightState.", "")

    def _state_color(self, state):

        if state == carla.TrafficLightState.Red:
            return carla.Color(255, 0, 0)

        if state == carla.TrafficLightState.Yellow:
            return carla.Color(255, 255, 0)

        if state == carla.TrafficLightState.Green:
            return carla.Color(0, 255, 0)

        return carla.Color(255, 255, 255)
