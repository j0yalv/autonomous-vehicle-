class SpeedSmoother:

    def __init__(
        self,
        initial_speed=0.0,
        max_accel_rate=1.2,
        max_decel_rate=4.0,
        throttle_rate=2.0,
        brake_rate=5.0
    ):

        self.target_speed = initial_speed
        self.throttle = 0.0
        self.brake = 0.0
        self.max_accel_rate = max_accel_rate
        self.max_decel_rate = max_decel_rate
        self.throttle_rate = throttle_rate
        self.brake_rate = brake_rate

    def smooth_target_speed(self, desired_speed, dt, force_stop=False):

        if force_stop:
            max_change = self.max_decel_rate * 2.0 * dt
        elif desired_speed > self.target_speed:
            max_change = self.max_accel_rate * dt
        else:
            max_change = self.max_decel_rate * dt

        delta = desired_speed - self.target_speed
        delta = max(-max_change, min(max_change, delta))

        self.target_speed += delta

        if desired_speed <= 0.0 and self.target_speed < 0.05:
            self.target_speed = 0.0

        print(
            f"Speed smoothing | desired: {desired_speed:.2f}m/s | "
            f"smoothed: {self.target_speed:.2f}m/s | "
            f"force_stop: {force_stop}"
        )

        return self.target_speed

    def smooth_control(self, throttle, brake, dt, emergency_brake=False):

        if emergency_brake:
            self.throttle = 0.0
            self.brake = brake

            print(
                f"Brake decision | emergency brake: {brake:.2f}"
            )

            return self.throttle, self.brake

        throttle = self._rate_limit(
            self.throttle,
            throttle,
            self.throttle_rate * dt
        )

        brake = self._rate_limit(
            self.brake,
            brake,
            self.brake_rate * dt
        )

        if brake > 0.05:
            throttle = 0.0
        elif throttle > 0.05:
            brake = 0.0

        self.throttle = throttle
        self.brake = brake

        print(
            f"Brake decision | throttle: {throttle:.2f} | "
            f"brake: {brake:.2f}"
        )

        return throttle, brake

    def _rate_limit(self, current_value, desired_value, max_change):

        delta = desired_value - current_value
        delta = max(-max_change, min(max_change, delta))

        return current_value + delta
