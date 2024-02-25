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

from sympy import symbols, Matrix, S, sin, cos, simplify, Matrix, MatrixSymbol, pprint
from sympy import lambdify
from collections import namedtuple
import dill
import hashlib


def run_calcs():
    l0, l1, l2, l3 = symbols("l:4", real=True)
    q = MatrixSymbol("q", 1, 3)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    def dh(a, d, alpha, t):
        return Matrix(
            [
                [cos(t), -sin(t) * cos(alpha), sin(t) * sin(alpha), a * cos(t)],
                [sin(t), cos(t) * cos(alpha), -cos(t) * sin(alpha), a * sin(t)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    a10 = dh(0, l0, -S.Pi / 2, S.Pi / 2 + q1)
    a21 = dh(l2, -l1, 0, q2 - S.Pi / 2)
    a32 = dh(l3, 0, 0, q3 - S.Pi / 2)
    ae3 = Matrix([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

    a10 = a10
    a20 = a10 * a21
    a30 = a20 * a32
    ae0 = a30 * ae3

    p1 = a10[:3, -1]
    p2 = a20[:3, -1]
    pe = ae0[:3, -1]

    b0 = Matrix([0, 0, 1])
    b1 = a20[:3, 2]
    b2 = a30[:3, 2]

    j1 = Matrix([b0.cross(pe), b0])
    j2 = Matrix([b1.cross(pe - p1), b1])
    j3 = Matrix([b2.cross(pe - p2), b2])

    j = j1.row_join(j2).row_join(j3)

    jlinear = j[:3, :]

    p1_after = (a10 @ Matrix([[0], [0], [-l1], [1]]))[:3, 0]

    exports = "p1 p2 pe jlinear p1_after".split()
    variables = locals()
    exports = {export: variables[export] for export in exports}
    exports = {
        k: export.subs([(l0, 0.15), (l1, 0.20), (l2, 0.30), (l3, 0.30)])
        for k, export in exports.items()
    }
    exports = {k: lambdify(q, export) for k, export in exports.items()}
    # ensures that if the code of this function changes the cached won't be used.
    exports["SELF_HASH"] = hashlib.sha256(run_calcs.__code__.co_code + dill.dumps(run_calcs.__code__.co_consts)).digest()

    export_type = namedtuple("RobotFunctions", exports.keys())
    return export_type(**exports)


def get_robot_functions():
    # caching robot_functions because it requires much time
    robot_functions = None
    try:
        with open("robot_functions.dill", "rb") as f:
            robot_functions = dill.load(f)

        # check if code of 'run_calcs' changed since the last cache
        current_hash = hashlib.sha256(run_calcs.__code__.co_code + dill.dumps(run_calcs.__code__.co_consts)).digest()
        if robot_functions.SELF_HASH != current_hash:
            robot_functions = None
    except (OSError, EOFError, dill.PickleError):
        pass

    if robot_functions is None:
        robot_functions = run_calcs()
        with open("robot_functions.dill", "wb") as f:
            dill.dump(robot_functions, f)

    return robot_functions
