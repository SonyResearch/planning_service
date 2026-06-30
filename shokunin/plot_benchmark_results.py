import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import ListedColormap


def read_results_yaml_file(filepath):
    with open(filepath, "r") as file:
        data = yaml.safe_load(file)
        return data["results"]


def plot_results(data, filename) -> None:
    xs = np.array([entry["x"] for entry in data])
    ys = np.array([entry["y"] for entry in data])
    yaw = np.array([entry["yaw"] for entry in data])
    durations = np.array([entry["duration"] for entry in data])
    configuration_twirl = np.array([entry["configuration_twirl"] for entry in data])
    success = np.array([entry["success"] for entry in data])

    x_yaw = xs + 0.004 * np.cos(yaw)
    y_yaw = ys + 0.004 * np.sin(yaw)

    # Make 3 plots in a subplot
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    # Plot Duration
    sc = ax[0].scatter(x_yaw, y_yaw, c=durations, cmap="viridis", s=80, edgecolor="k")
    plt.colorbar(sc, ax=ax[0], label="Duration (s)")
    ax[0].set_xlabel("x")
    ax[0].set_ylabel("y")
    ax[0].set_title("Duration vs (x, y)")
    # Plot Configuration Twirl
    sc2 = ax[1].scatter(x_yaw, y_yaw, c=configuration_twirl, cmap="plasma", s=80, edgecolor="k")
    plt.colorbar(sc2, ax=ax[1], label="Configuration Twirl (rad)")
    ax[1].set_xlabel("x")
    ax[1].set_ylabel("y")
    ax[1].set_title("Configuration Twirl vs (x, y)")
    # Plot Success
    cmap = ListedColormap(["red", "green"])
    sc3 = ax[2].scatter(x_yaw, y_yaw, c=success, cmap=cmap, s=80, edgecolor="k")
    # Add colorbar with ticks at 0 and 1
    cbar = plt.colorbar(sc3, ax=ax[2], ticks=[0, 1])
    cbar.ax.set_yticklabels(["Failure", "Success"])

    ax[2].set_xlabel("x")
    ax[2].set_ylabel("y")
    ax[2].set_title("Success vs (x, y)")
    # Save the figure
    plt.tight_layout()
    plt.savefig(f"{filename}", dpi=300)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Visualize YAML results")
    parser.add_argument("input_file", type=str, help="Path to the input YAML file")
    parser.add_argument("output_file", type=str, help="Path to save the output plot")
    args = parser.parse_args()

    try:
        data = read_results_yaml_file(args.input_file)
        plot_results(data, args.output_file)
        sys.exit(0)  # success
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)  # failure
