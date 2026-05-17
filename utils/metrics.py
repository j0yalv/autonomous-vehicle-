class Metrics:

    def __init__(self):

        self.speeds = []
        self.heading_errors = []
        self.steers = []

    def record(self, speed, heading_error, steer):

        self.speeds.append(speed)
        self.heading_errors.append(abs(heading_error))
        self.steers.append(abs(steer))

    def summary(self):

        avg_speed = sum(self.speeds) / len(self.speeds)

        avg_heading_error = (
            sum(self.heading_errors) / len(self.heading_errors)
        )

        avg_steer = sum(self.steers) / len(self.steers)

        print("\n------ PERFORMANCE METRICS ------")

        print(f"Average Speed: {avg_speed:.2f}")

        print(
            f"Average Heading Error: "
            f"{avg_heading_error:.4f}"
        )

        print(
            f"Average Steering Magnitude: "
            f"{avg_steer:.4f}"
        )   