"""Rooms for the 2.5D simulator, by name (sim2d_node / sim_depth_node parameter `world`, launch argument world:=).

  room   the SLAM / dock room of core/sim2d.default_room (tall cabinet, pillar, dock)
  toy    a 0.20 × 0.15 × 0.08 m toy box on the straight path
  table  a 1.0 × 0.5 m coffee table, top Z0.40–0.45, four Ø0.04 legs: the robot fits between the legs, its head not
         under the top
  chair  a chair, seat Z0.42–0.45, four Ø0.025 legs, back Z0.45–0.90 on one side (the only part in the scan plane)
  dock   `room` plus small low objects on the way to the dock (same scan as `room`: its saved map still fits)
  door   a wall across the room, its 0.70 m doorway blocked by a 0.30 m ottoman: the only way to the goal is blocked
Everything except the walls, the dock's back plate and the chair back is below the LiDAR plane (Z0.58)."""
import math

from .dock_nav import Pose2D
from .sim2d import Block, Cylinder, Room, box, default_room, dock_body

DOCK = Pose2D(3.0, 1.75, math.pi / 2)      # north wall, as in the default room


def _open_room(*extra_walls):
    """6 × 4 m, a pillar and a shelf for SLAM to hold on to, the dock on the north wall."""
    return box(-1.0, -2.0, 5.0, 2.0) + box(3.6, -1.5, 3.9, -1.2) + box(-0.8, 1.5, 0.2, 1.9) + tuple(extra_walls)


def table(name, cx, cy, size_x, size_y, top_z0, top_z1, leg_r, inset):
    hx, hy = size_x / 2, size_y / 2
    legs = tuple(Cylinder(f'{name}_leg', cx + sx * (hx - inset - leg_r), cy + sy * (hy - inset - leg_r), leg_r, 0.0,
                          top_z0) for sx in (-1, 1) for sy in (-1, 1))
    return (Block(f'{name}_top', cx - hx, cy - hy, cx + hx, cy + hy, top_z0, top_z1),) + legs


def chair(name, cx, cy, size=0.45, seat=(0.42, 0.45), back_top=0.90, leg_r=0.0125, inset=0.02):
    """Back on the +y side; its two legs run up to the top of the back."""
    h = size / 2
    legs = tuple(Cylinder(f'{name}_leg', cx + sx * (h - inset - leg_r), cy + sy * (h - inset - leg_r), leg_r, 0.0,
                          back_top if sy > 0 else seat[0]) for sx in (-1, 1) for sy in (-1, 1))
    return (Block(f'{name}_seat', cx - h, cy - h, cx + h, cy + h, *seat),
            Block(f'{name}_back', cx - h, cy + h - 0.03, cx + h, cy + h, seat[1], back_top)) + legs


def toy_room():
    return Room(_open_room(), DOCK, (dock_body(DOCK), Block('toy_box', 1.40, -0.075, 1.60, 0.075, 0.0, 0.08)))


def table_room():
    return Room(_open_room(), DOCK, (dock_body(DOCK),) + table('table', 1.80, 0.0, 0.5, 1.0, 0.40, 0.45, 0.02, 0.03))


def chair_room():
    return Room(_open_room(), DOCK, (dock_body(DOCK),) + chair('chair', 1.80, 0.0))


def dock_room():
    r = default_room()
    clutter = (Block('shoe_box', 2.15, 1.20, 2.45, 1.32, 0.0, 0.10),
               Cylinder('ball', 2.45, 0.75, 0.07, 0.0, 0.14),
               Block('toy_car', 3.45, 1.02, 3.65, 1.14, 0.0, 0.07))
    return Room(r.walls, r.dock, r.obstacles + clutter)


def door_room():
    wall = box(1.95, -2.0, 2.05, -0.35) + box(1.95, 0.35, 2.05, 2.0)
    return Room(_open_room(*wall), DOCK, (dock_body(DOCK), Block('ottoman', 1.75, -0.40, 2.25, 0.40, 0.0, 0.30)))


WORLDS = {'room': default_room, 'toy': toy_room, 'table': table_room, 'chair': chair_room, 'dock': dock_room,
          'door': door_room}


def world(name):
    if name not in WORLDS:
        raise ValueError(f'world {name!r}: expected one of {", ".join(WORLDS)}')
    return WORLDS[name]()
