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

import tkinter as tk
from tkinter import ttk, messagebox


def read_gui():
    r, dt = None, None

    def submit_values():
        nonlocal r, dt
        dt_value = dt_entry.get()
        r_value = r_entry.get()
        try:
            dt = float(dt_value)
            r = float(r_value)
            if dt <= 0:
                raise ValueError("Dt must be positive.")
            if r <= 0:
                raise ValueError("R must be positive.")
        except ValueError as e:
            messagebox.showerror("Validation Error", str(e))
            return
        root.destroy()

    root = tk.Tk()
    root.title("Input Dt and R")

    frame = ttk.Frame(root, padding="10")
    frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

    dt_label = ttk.Label(frame, text="Enter Dt (time in seconds, e.g. 2 sec) :")
    dt_label.grid(row=0, column=0, sticky=tk.W)
    dt_entry = ttk.Entry(frame, width=15)
    dt_entry.grid(row=0, column=1)

    r_label = ttk.Label(frame, text="Enter R (radius in meters, e.g. 0.075 m):")
    r_label.grid(row=1, column=0, sticky=tk.W)
    r_entry = ttk.Entry(frame, width=15)
    r_entry.grid(row=1, column=1)

    submit_button = ttk.Button(frame, text="Go", command=submit_values)
    submit_button.grid(row=2, pady=10, column=0, columnspan=2)

    root.mainloop()
    return r, dt
