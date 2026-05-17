import matplotlib.pyplot as plt


class PlotMetrics:

    def __init__(self):

        self.speeds = []
        self.heading_errors = []
        self.steers = []

    def record(self, speed, heading_error, steer):

        self.speeds.append(speed)

        self.heading_errors.append(
            abs(heading_error)
        )

        self.steers.append(abs(steer))

    def plot(self):

        plt.figure(figsize=(12, 6))

        plt.plot(self.speeds)

        plt.title("Vehicle Speed")

        plt.xlabel("Time Step")

        plt.ylabel("Speed")

        plt.grid(True)

        plt.show()

        plt.figure(figsize=(12, 6))

        plt.plot(self.heading_errors)

        plt.title("Heading Error")

        plt.xlabel("Time Step")

        plt.ylabel("Error")

        plt.grid(True)

        plt.show()

        plt.figure(figsize=(12, 6))

        plt.plot(self.steers)

        plt.title("Steering Values")

        plt.xlabel("Time Step")

        plt.ylabel("Steer")

        plt.grid(True)

        plt.show()