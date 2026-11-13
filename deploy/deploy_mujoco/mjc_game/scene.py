import mujoco
import numpy as np

from .settings import ARROW_LENGTH, GOAL_RADIUS


def add_geom(scn, geom_type, size, pos, rgba, label=""):
    """Appends a geom to the viewer's user scene."""
    if scn.ngeom >= scn.maxgeom:
        return None
    geom = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(geom, geom_type, np.asarray(size, dtype=np.float64), np.asarray(pos, dtype=np.float64),
                        np.eye(3).flatten(), np.asarray(rgba, dtype=np.float32))
    if label:
        geom.label = label
    scn.ngeom += 1
    return geom


def add_arrow(scn, start, end, width, rgba):
    """Appends an arrow from start to end to the viewer's user scene."""
    geom = add_geom(scn, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), rgba)
    if geom is not None:
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width,
                             np.asarray(start, dtype=np.float64), np.asarray(end, dtype=np.float64))


def draw_scene(scn, game, trail, base_pos, robot_heading, target_heading, hud):
    """Goal, trail, heading arrows and the HUD label, redrawn every control step."""
    scn.ngeom = 0

    # Goal: disc on the floor and a pole with a label
    gx, gy = game.goal
    add_geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [GOAL_RADIUS, 0.005, 0], [gx, gy, 0.005], [1.0, 0.5, 0.0, 0.5])
    add_geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.03, 0.75, 0], [gx, gy, 0.75], [1.0, 0.5, 0.0, 1.0])
    add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.08, 0, 0], [gx, gy, 1.55], [1.0, 0.8, 0.0, 1.0],
             label=f"GOAL {game.score + 1}")

    # Trail of the path walked
    for x, y in trail:
        add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.03, 0, 0], [x, y, 0.02], [0.3, 0.7, 1.0, 0.6])

    # Heading arrows at floor level: robot heading (blue), target heading (green)
    start = np.array([base_pos[0], base_pos[1], 0.05])
    for heading, rgba in ((robot_heading, [0.2, 0.4, 1.0, 1.0]), (target_heading, [0.1, 1.0, 0.3, 1.0])):
        end = start + ARROW_LENGTH * np.array([np.cos(heading), np.sin(heading), 0.0])
        add_arrow(scn, start, end, 0.03, rgba)

    # HUD label above the robot
    add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.001, 0, 0], [base_pos[0], base_pos[1], base_pos[2] + 0.9],
             [0, 0, 0, 0], label=hud)
