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
import scipy as sp
from matplotlib import pyplot as plt
import matplotlib
matplotlib.rc('font',family='CMU Serif')


class SnappingCursor:
    def __init__(self, ax, line):
        self.ax = ax
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        self.horizontal_line = ax.axhline(color='k', lw=0.8, ls='--')
        self.vertical_line = ax.axvline(color='k', lw=0.8, ls='--')
        self.x, self.y = line.get_data()
        self._last_index = None
        self.text = ax.text(0.6, 0.9, '', transform=ax.transAxes, fontname="CMU Serif", fontsize=12)
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)

    def set_cross_hair_visible(self, visible):
        need_redraw = self.horizontal_line.get_visible() != visible
        self.horizontal_line.set_visible(visible)
        self.vertical_line.set_visible(visible)
        self.text.set_visible(visible)
        return need_redraw

    def on_mouse_move(self, event):
        if not event.inaxes:
            self._last_index = None
            need_redraw = self.set_cross_hair_visible(False)
            if need_redraw:
                self.ax.figure.canvas.draw()
        else:
            self.set_cross_hair_visible(True)
            x, y = event.xdata, event.ydata
            index = min(np.searchsorted(self.x, x), len(self.x) - 1)
            if index == self._last_index:
                return
            self._last_index = index
            x = self.x[index]
            y = self.y[index]
            # update the line positions
            self.horizontal_line.set_ydata([y])
            self.vertical_line.set_xdata([x])
            self.text.set_text(f'$α = {x:1.2f}, υ_{{cruise}}={y:1.2f}$')
            self.ax.figure.canvas.draw()


    def on_click(self, event):
        x, y = event.xdata, event.ydata
        index = min(np.searchsorted(self.x, x), len(self.x) - 1)
        x = self.x[index]
        y = self.y[index]
        self._selection = (x,y) 
        plt.close()

    @property
    def selection(self):
        return self._selection

def pareto(R, Dt):
    u = np.linspace(1e-3+(np.pi+2)*R/Dt, 5 * (np.pi+2)*R/Dt, 1000)
    a = 2*u/(Dt - (np.pi+2)*R/u)

    feas = (u**2/a <= 2*R) & (a > 0)
    a = a[feas == True]
    u = u[feas == True]
    
    a,u = zip(*sorted((ai,ui) for ai,ui in zip(a,u)))

    tck = sp.interpolate.splrep(a, u)
    a_line = np.linspace(min(a), max(a), 1000)
    u_line = sp.interpolate.splev(a_line, tck)

    fig, ax = plt.subplots()
    ln, = ax.plot(a_line, u_line)
    ax.set_xlabel(r'$α$ [m/s^2]', fontname="CMU Serif", fontsize=12)
    ax.set_ylabel(r'$υ_{cruise}$ [m/s]', fontname="CMU Serif", fontsize=12)
    plt.savefig("pareto.pdf")
    snap_cursor = SnappingCursor(ax, ln)
    fig.canvas.mpl_connect('motion_notify_event', snap_cursor.on_mouse_move)
    fig.canvas.mpl_connect('button_press_event', snap_cursor.on_click)
    plt.title("Pareto Front: Click to select desired point.\nYou are advised to peak a point that averages both. Otherwise the robot might be unable to follow the trajectory", wrap=True)
    plt.tight_layout()
    plt.show()
    return snap_cursor.selection

