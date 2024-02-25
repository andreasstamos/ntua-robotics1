# Semester Project - Robotics 1 - NTUA
# Copyright (C) 2024  Andreas Stamos
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np

from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
import matplotlib

matplotlib.rc("font", family="CMU Serif")

VISUALIZE_RATE = 30


def visualize(positions, trajectory_correct, sampling_rate):
    positions = [np.zeros(positions[0].shape), *positions]
    trajectory = positions[-1]

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    (robot,) = ax.plot(
        [],
        [],
        [],
        label="robot simulation",
        marker="o",
        markerfacecolor="red",
        markeredgecolor="red",
        color="blue",
    )
    (ln,) = ax.plot([], [], [], label="simulated trajectory", color="tab:green")
    (ln_c,) = ax.plot([], [], [], "--", label="planned trajectory", color="tab:orange")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_aspect("equal")

    xmin = min(np.min(position[0, :]) for position in positions)
    ymin = min(np.min(position[1, :]) for position in positions)
    zmin = min(np.min(position[2, :]) for position in positions)
    xmax = max(np.max(position[0, :]) for position in positions)
    ymax = max(np.max(position[1, :]) for position in positions)
    zmax = max(np.max(position[2, :]) for position in positions)

    ax.set(xlim3d=(xmin, xmax), xlabel="$x$")
    ax.set(ylim3d=(ymin, ymax), ylabel="$y$")
    ax.set(zlim3d=(zmin, zmax), zlabel="$z$")

    def update(n, ln, ln_c, robot):
        n = int(n * sampling_rate / VISUALIZE_RATE)

        robot.set_data(
            [positions[i][0][n] for i in range(len(positions))],
            [positions[i][1][n] for i in range(len(positions))],
        )
        robot.set_3d_properties([positions[i][2][n] for i in range(len(positions))])

        ln.set_data(trajectory[:2, :n])
        ln.set_3d_properties(trajectory[2, :n])

        ln_c.set_data(trajectory_correct[:2, :n])
        ln_c.set_3d_properties(trajectory_correct[2, :n])
        return ln, ln_c, robot

    ani = FuncAnimation(
        fig,
        update,
        trajectory_correct.shape[1] * VISUALIZE_RATE // sampling_rate,
        fargs=(ln, ln_c, robot),
        interval=1000 / VISUALIZE_RATE,
    )
    ani.save("animation.mp4", dpi=300)

    ax.legend()
    plt.show()


def path_plots(points, us_3d, s, us, Dt, HZ, prefix, fn_prefix):
    t = np.linspace(0, Dt, s.shape[0])

    plt.plot(t, us)
    plt.xlabel(r"$t$ [s]", fontname="CMU Serif", fontsize=12)
    plt.ylabel(r"$|υ|$ [m/s]", fontname="CMU Serif", fontsize=12)
    plt.savefig(f"{fn_prefix}_u_norm.pdf")
    plt.title(f"{prefix}: Norm of speed - time")
    plt.show()

    plt.plot(t, s)
    plt.xlabel(r"$t$ [s]", fontname="CMU Serif", fontsize=12)
    plt.ylabel(r"$|s|$ [m]", fontname="CMU Serif", fontsize=12)
    plt.savefig(f"{fn_prefix}_s_norm.pdf")
    plt.title(f"{prefix}: Distance travelled - time")
    plt.show()

    EVERY = int(Dt * HZ // 20)
    plt.quiver(
        points[1, :][::EVERY],
        points[2, :][::EVERY],
        us_3d[1, :][::EVERY],
        us_3d[2, :][::EVERY],
    )
    plt.axis("equal")
    plt.xlabel(r"$Y$ [m]", fontname="CMU Serif", fontsize=12)
    plt.ylabel(r"$Z$ [m]", fontname="CMU Serif", fontsize=12)
    plt.savefig(f"{fn_prefix}_speed_vect.pdf")
    plt.title(f"{prefix}: Speed as a vector field of position")
    plt.show()

    fig, ax = plt.subplots(1, 3, sharex=True, sharey=True, figsize=(13, 7))
    ax[0].plot(t, us_3d[0, :])
    ax[1].plot(t, us_3d[1, :])
    ax[2].plot(t, us_3d[2, :])
    for i in range(3):
        ax[i].set_xlabel(r"$t$ [s]", fontname="CMU Serif", fontsize=12)
    ax[0].set_title(r"$υ_x$ [m/s]", fontname="CMU Serif", fontsize=12)
    ax[1].set_title(r"$υ_y$ [m/s]", fontname="CMU Serif", fontsize=12)
    ax[2].set_title(r"$υ_z$ [m/s]", fontname="CMU Serif", fontsize=12)
    plt.savefig(f"{fn_prefix}_speeds.pdf")
    plt.suptitle(f"{prefix}: Cartesian coordinates of speed - time")
    plt.show()

    fig, ax = plt.subplots(1, 3, sharex=True, figsize=(13, 7))
    ax[0].plot(t, points[0, :])
    ax[1].plot(t, points[1, :])
    ax[2].plot(t, points[2, :])
    for i in range(3):
        ax[i].set_xlabel(r"$t$ [s]", fontname="CMU Serif", fontsize=12)
    ax[0].set_title(r"$x$ [m]", fontname="CMU Serif", fontsize=12)
    ax[1].set_title(r"$y$ [m]", fontname="CMU Serif", fontsize=12)
    ax[2].set_title(r"$z$ [m]", fontname="CMU Serif", fontsize=12)
    plt.savefig(f"{fn_prefix}_points.pdf")
    plt.suptitle(f"{prefix}: Cartesian coordinates of end point - time")
    plt.show()
