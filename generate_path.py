import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

class Path:
    def __init__(self):
        self.original_path = pd.DataFrame()

        
    def generate_path(self):
        """generates path points for testing purposes"""
        
        line_x = np.arange(0, 50, 0.5)
        line_y = [math.sin(ix / 5.0) * ix / 2.0 for ix in line_x]
        velocity = np.abs(np.gradient(line_y))

        min_velocity = 3
        max_velocity = 26
        normalized_velocity = min_velocity + (velocity - np.min(velocity)) *\
        (max_velocity - min_velocity) / (np.max(velocity) - np.min(velocity))
        path_points_df = pd.DataFrame({"x": line_x, "y": line_y,
                                    "v": normalized_velocity})
        self.original_path = path_points_df
        return path_points_df
    
    def plot_path(self, final_data):
        """plots the original path and vehicle trajectory"""
        original_path = self.original_path
    
        fig, ax = plt.subplots()
        ax.plot(original_path["x"], original_path["y"], label="Original Path",
                color="black")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        
        # plot vehicle path
        for i in range(len(final_data.x)):
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(final_data.x[i], final_data.y[i], ".", color="red",
                    label="Vehicle current location")
            plt.axis("equal")
            plt.title(f"vehicle speed: {final_data.v[i]}")
            plt.grid(True)
            plt.pause(0.1)
            if i == 0:
                plt.legend()
        plt.show()