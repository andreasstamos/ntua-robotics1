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
import dill
from visualize import visualize, path_plots
from symbolic import get_robot_functions
from path import find_trajectory
from pareto import pareto
from gui import read_gui
import tkinter

print("Running: Calculate symbolically robot functions...")
rf = get_robot_functions()

HZ = 200
DT = 1 / HZ


class Robot:
    _q = np.zeros(3)
    _qd = np.zeros(3)
    def step(self, qdd):
        # assume constant acceleration during dt
        self._q += self._qd * DT + 1 / 2 * qdd * DT**2
        self._qd += qdd * DT

    def position(self):
        return rf.pe(np.atleast_2d(self._q)).reshape((3,))

    def q(self):
        return self._q

    def velocity_lin(self):
        return rf.jlinear(np.atleast_2d(self._q)) @ self._qd

    def position_all_joints(self):
        return [
            f(np.atleast_2d(self._q)).reshape((3,))
            for f in [rf.p1, rf.p1_after, rf.p2, rf.pe]
        ]

def inverse_kinematics(points):
    # newton-raphson for inverse kinematics
    q = np.zeros(points.shape)
    q[:, 0] = [0, 0, 0]
    TOL = 1e-3
    for i in range(1, points.shape[1]):
        q[:, i] = q[:, i - 1]
        while True:
            dp = points[:, i] - rf.pe(np.atleast_2d(q[:, i])).flatten()
            if np.max(np.abs(dp)) < TOL:
                break
            dx = np.linalg.solve(rf.jlinear(np.atleast_2d(q[:, i])), dp)
            q[:, i] += dx
    return q


class PID:
    def __init__(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._u = np.zeros(3)
        self._e = [np.zeros(3), np.zeros(3), np.zeros(3)]

    def step(self, setpoint, current):
        self._e[2] = self._e[1]
        self._e[1] = self._e[0]
        self._e[0] = setpoint - current
        self._u += (
            (self._kp + self._ki * DT + self._kd / DT) * self._e[0]
            + (-self._kp - 2 * self._kd / DT) * self._e[1]
            + self._kd / DT * self._e[2]
        )
        return self._u


def main():
    print("Running: Read from GUI...")
    R, Dt = read_gui()

    A, U = pareto(R, Dt)
    print("Running: Trajectory planning...")
    trajectory, trajectory_us_3d, trajectory_s, trajectory_us = find_trajectory(R, Dt, U, A, HZ)
    print("Running: Plotting trajectory plans...")
    tkinter.messagebox.showinfo("Trajectory Plan Figures", "The figures that follow are the trajectory plan. NOT from simulation. These will be displayed after the simulation.")
    path_plots(trajectory, trajectory_us_3d, trajectory_s, trajectory_us, Dt, HZ, prefix="Trajectory Plan", fn_prefix="plan")
    
    print("Running: Inverse kinematics...")
    q = inverse_kinematics(trajectory)

    print("Running: Simulation...")
    robot = Robot()
    res = [np.zeros(q.shape) for _ in range(4)]
    sim_trajectory_us_3d = np.zeros(q.shape)
    pid = PID(kp=1200, ki=8000, kd=60)
    for i in range(q.shape[1]):
        u = pid.step(setpoint=q[:, i], current=robot.q())
        robot.step(u)
        for arr, pos in zip(res, robot.position_all_joints()):
            arr[:, i] = pos
        sim_trajectory_us_3d[:,i] = robot.velocity_lin()

    print("Running: Simulation Visualization...")
    visualize(positions=res, trajectory_correct=trajectory, sampling_rate=HZ)
    tkinter.messagebox.showinfo("Simulation Figures", "The figures that follow are from the actual simulation.")

    sim_trajectory = res[-1]
    sim_trajectory_s = np.cumsum(np.sqrt(np.sum(np.diff(sim_trajectory, prepend=sim_trajectory[:,0][:, np.newaxis], axis=1)**2, axis=0)), axis=0)
    sim_trajectory_us = np.sqrt(np.sum(sim_trajectory_us_3d**2, axis=0)) 
    path_plots(sim_trajectory, sim_trajectory_us_3d, sim_trajectory_s, sim_trajectory_us, Dt, HZ, prefix="Simulation", fn_prefix="sim")


if __name__ == "__main__":
    main()
