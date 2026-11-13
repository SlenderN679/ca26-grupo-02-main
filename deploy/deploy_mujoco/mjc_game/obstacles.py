import colorsys

import mujoco
import numpy as np

from .math_utils import quat_from_yaw
from .settings import (HIT_COOLDOWN, PILLAR_HALF_HEIGHT, PILLAR_ORBIT, PILLAR_PERIOD, PILLAR_RADIUS,
                       SWEEPER_AT, SWEEPER_HEIGHT, SWEEPER_PERIOD, SWEEPER_SIZE,
                       WALL_AMPLITUDE, WALL_AT, WALL_PERIOD, WALL_SIZE)


def build_model(xml_path):
    """Loads the scene and adds the obstacle bodies (mocap: moved by the script, solid for the robot)."""
    spec = mujoco.MjSpec()
    spec.from_file(xml_path)

    def add_obstacle(name, shapes):
        body = spec.worldbody.add_body()
        body.name = name
        body.mocap = True
        body.pos = [0, 0, -10]  # out of the way until placed
        for i, (geom_type, size, pos, rgba) in enumerate(shapes):
            geom = body.add_geom()
            geom.name = f"{name}_{i}"
            geom.type = geom_type
            geom.size = size
            geom.pos = pos
            geom.rgba = rgba
        return body

    red = [0.9, 0.2, 0.2, 1.0]
    add_obstacle("wall", [(mujoco.mjtGeom.mjGEOM_BOX, WALL_SIZE, [0, 0, 0], red)])
    add_obstacle("sweeper", [
        (mujoco.mjtGeom.mjGEOM_BOX, SWEEPER_SIZE, [0, 0, 0], [0.9, 0.3, 0.8, 1.0]),
        # post from the floor to just above the bar (body frame is at the bar centre)
        (mujoco.mjtGeom.mjGEOM_CYLINDER, [0.08, (SWEEPER_HEIGHT + 0.15) / 2, 0],
         [0, 0, (SWEEPER_HEIGHT + 0.15) / 2 - SWEEPER_HEIGHT], [0.5, 0.5, 0.5, 1.0]),
    ])
    for i in range(2):
        add_obstacle(f"pillar{i}", [(mujoco.mjtGeom.mjGEOM_CYLINDER, [PILLAR_RADIUS, PILLAR_HALF_HEIGHT, 0],
                                     [0, 0, 0], [0.9, 0.6, 0.1, 1.0])])
    return spec.compile()


class Obstacles:
    """Moves the obstacles around the current goal and detects hits on the robot."""

    NAMES = ["wall", "sweeper", "pillar0", "pillar1"]

    def __init__(self, m):
        body_ids = [m.body(name).id for name in self.NAMES]
        self.mocap = {name: m.body_mocapid[i] for name, i in zip(self.NAMES, body_ids)}
        self.geom_owner = {g: m.geom_bodyid[g] for g in range(m.ngeom) if m.geom_bodyid[g] in body_ids}
        self.robot_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] != 0 and g not in self.geom_owner}
        self.last_hit = {i: -np.inf for i in body_ids}
        self.start = np.zeros(2)
        self.goal = np.zeros(2)
        self.path_angle = 0.0
        self.m = m
        self.rng = np.random.default_rng()
        # Geoms recoloured each round, grouped per obstacle (the sweeper post stays grey)
        self.color_geoms = [[g for g, b in self.geom_owner.items() if b == body and m.geom(g).name != "sweeper_1"]
                            for body in body_ids]

    def randomize_colors(self):
        """A random bright colour per obstacle."""
        for geoms in self.color_geoms:
            rgb = colorsys.hsv_to_rgb(self.rng.uniform(0.0, 1.0), self.rng.uniform(0.6, 1.0), self.rng.uniform(0.7, 1.0))
            for g in geoms:
                self.m.geom_rgba[g] = [*rgb, 1.0]

    def place(self, robot_xy, goal_xy):
        self.randomize_colors()
        self.start = np.array(robot_xy, dtype=float)
        self.goal = np.array(goal_xy, dtype=float)
        delta = self.goal - self.start
        self.path_angle = np.arctan2(delta[1], delta[0])

    def update(self, d, t):
        direction = np.array([np.cos(self.path_angle), np.sin(self.path_angle)])
        side = np.array([-direction[1], direction[0]])

        # Sliding wall across the path, like a door opening and closing
        wall_xy = self.start + WALL_AT * (self.goal - self.start) + WALL_AMPLITUDE * np.sin(2 * np.pi * t / WALL_PERIOD) * side
        d.mocap_pos[self.mocap["wall"]] = [wall_xy[0], wall_xy[1], WALL_SIZE[2]]
        d.mocap_quat[self.mocap["wall"]] = quat_from_yaw(self.path_angle)

        # Sweeper bar spinning around its post
        post_xy = self.start + SWEEPER_AT * (self.goal - self.start)
        d.mocap_pos[self.mocap["sweeper"]] = [post_xy[0], post_xy[1], SWEEPER_HEIGHT]
        d.mocap_quat[self.mocap["sweeper"]] = quat_from_yaw(2 * np.pi * t / SWEEPER_PERIOD)

        # Two pillars circling the goal on opposite sides
        for i in range(2):
            angle = 2 * np.pi * t / PILLAR_PERIOD + i * np.pi
            xy = self.goal + PILLAR_ORBIT * np.array([np.cos(angle), np.sin(angle)])
            d.mocap_pos[self.mocap[f"pillar{i}"]] = [xy[0], xy[1], PILLAR_HALF_HEIGHT]

    def new_hits(self, d, t):
        """Number of obstacles that started touching the robot (one per obstacle per cooldown)."""
        hits = 0
        for c in d.contact[:d.ncon]:
            for obstacle_geom, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if obstacle_geom in self.geom_owner and other in self.robot_geoms:
                    body = self.geom_owner[obstacle_geom]
                    if t - self.last_hit[body] >= HIT_COOLDOWN:
                        hits += 1
                    self.last_hit[body] = t
        return hits
