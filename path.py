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

from matplotlib import pyplot as plt
import numpy as np
import scipy as sp

def find_trajectory(R, Dt, U, A, HZ):
    def path(a, u, t, s):
        ts = np.arange(0, t, 1/HZ)
        us = np.zeros_like(ts)
        ds = np.zeros_like(ts)

        dt_acc = round(u / a * HZ)
        # fix rounding error to ensure 's' is reached
        a = s / ((dt_acc / HZ) * (t - dt_acc / HZ))

        dt_acc += 1
        us[:dt_acc] = a * ts[:dt_acc]
        us[dt_acc:-dt_acc] = us[dt_acc - 1]
        us[-dt_acc:] = us[:dt_acc][::-1]

        ds[:dt_acc] = 0.5 * a * ts[:dt_acc] ** 2
        ds[dt_acc:-dt_acc] = ds[dt_acc - 1] + us[dt_acc] * (ts[dt_acc:-dt_acc] - ts[dt_acc - 1])
        ds[-dt_acc:] = ds[-dt_acc - 1] + us[-dt_acc] * (ts[-dt_acc:] - ts[-dt_acc - 1]) - 0.5 * a * (ts[-dt_acc:] - ts[-dt_acc - 1]) ** 2

        ds = sp.integrate.cumulative_trapezoid(us,ts, initial=0)

        return us, ds

    t1 = U / A + np.pi * R / U
    t2 = U / A + 2 * R / U

    u1, s1 = path(A, U, t1, np.pi * R)
    u2, s2 = path(A, U, t2, 2 * R)
    us = np.hstack((u1, u2))

    s = np.hstack((s1, s2 + s1[-1]))

    phi = s1 / R
    points1 = np.array([0.20, -0.30, 0.45+R])[:, np.newaxis] + np.array(
        [np.zeros(phi.shape), R * np.sin(phi), -R * np.cos(phi)]
    )
    u1_3d = u1 * np.vstack((np.zeros(phi.shape), np.cos(phi), np.sin(phi)))

    points2 = np.array([0.20, -0.30, 0.45+2*R])[:, np.newaxis] + np.array(
        [np.zeros(s2.shape), np.zeros(s2.shape), -s2]
    )
    u2_3d = np.array([np.zeros(u2.shape), np.zeros(u2.shape), -u2])

    points = np.hstack((points1, points2))
    us_3d = np.hstack((u1_3d, u2_3d))

    return points, us_3d, s, us

