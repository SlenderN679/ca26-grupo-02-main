import collections
import time

import mujoco.viewer
import mujoco
import numpy as np
import os
# Repo root resolved from this file's location (deploy/deploy_mujoco/ -> repo root),
# so the script works regardless of the current working directory.
LEGGED_GYM_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import torch
import yaml

# Game rules, tunable settings and helpers live in the mjc_game package next to this script:
#   settings.py    game / obstacle / camera parameters (and a description of the game)
#   math_utils.py  quaternion helpers, wrap_to_pi, pd_control
#   game.py        Game: goals, score, timer, messages
#   obstacles.py   build_model (scene + obstacle bodies) and Obstacles (motion, hits)
#   scene.py       draw_scene: goal, trail, heading arrows and HUD in the viewer
from mjc_game.settings import (CAM_DISTANCE, CAM_ELEVATION, CAM_SMOOTHING, FALL_HEIGHT, FOLLOW_CAMERA,
                               INITIAL_YAW_DEG, SHOW_VIEWER_UI, TRAIL_LENGTH, TRAIL_PERIOD)
from mjc_game.math_utils import get_gravity_orientation, get_yaw, pd_control, quat_from_yaw, wrap_to_pi
from mjc_game.game import Game
from mjc_game.obstacles import Obstacles, build_model
from mjc_game.scene import draw_scene


if __name__ == "__main__":
    # get config file name from command line
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file
    with open(f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)

        default_angles = np.array(config["default_angles"], dtype=np.float32)

        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        
        cmd = np.array(config["cmd_init"], dtype=np.float32)

    # define context variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    counter = 0

    # Load robot model
    m = build_model(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt

    # load policy
    policy = torch.jit.load(policy_path)

    # Rotate the initial base orientation about the vertical axis
    yaw = np.radians(INITIAL_YAW_DEG)
    yaw_quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
    base_quat = np.zeros(4)
    mujoco.mju_mulQuat(base_quat, yaw_quat, d.qpos[3:7])
    d.qpos[3:7] = base_quat

    mujoco.mj_forward(m, d)
    robot_start_heading = get_yaw(d.qpos[3:7])
    target_heading = robot_start_heading

    game = Game()
    game.new_goal(d.qpos[0:2].copy(), d.time)
    obstacles = Obstacles(m)
    obstacles.place(d.qpos[0:2].copy(), game.goal)
    obstacles.update(d, d.time)
    hits = 0
    trail = collections.deque(maxlen=TRAIL_LENGTH)
    last_trail_time = -TRAIL_PERIOD
    pelvis_id = m.body("pelvis").id
    print("Game: walk the robot to the orange goal.")

    with mujoco.viewer.launch_passive(m, d, show_left_ui=SHOW_VIEWER_UI, show_right_ui=SHOW_VIEWER_UI) as viewer:
        if FOLLOW_CAMERA:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = pelvis_id
                viewer.cam.distance = CAM_DISTANCE
                viewer.cam.elevation = CAM_ELEVATION
                viewer.cam.azimuth = np.degrees(robot_start_heading)

        # Run until the viewer window is closed (simulation_duration from the config is ignored).
        start = time.time()
        while viewer.is_running():
            tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
            d.ctrl[:] = tau
            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            counter += 1
            if counter % control_decimation == 0:
                # Apply control signal here.

                # create observation
                qj = d.qpos[7:]
                dqj = d.qvel[6:]
                quat = d.qpos[3:7]
                omega = d.qvel[3:6]

                qj = (qj - default_angles) * dof_pos_scale
                dqj = dqj * dof_vel_scale
                gravity_orientation = get_gravity_orientation(quat)
                omega = omega * ang_vel_scale

                # ------------------------------------------------------------------
                # Implement the control here: from the received IMU data compute
                #   vx             forward speed command [m/s]
                #   yaw_cmd        yaw rate command [rad/s]
                #   target_heading heading the robot should face [rad] (green arrow)
                # Useful: get_yaw(quat) (robot heading [rad]), robot_start_heading, wrap_to_pi()
                # ------------------------------------------------------------------
                vx = 0.0
                yaw_cmd = 0.0

                cmd_obs = np.array([vx, 0.0, yaw_cmd], dtype=np.float32)

                period = 0.8
                count = counter * simulation_dt
                phase = count % period / period
                sin_phase = np.sin(2 * np.pi * phase)
                cos_phase = np.cos(2 * np.pi * phase)

                obs[:3] = omega
                obs[3:6] = gravity_orientation
                obs[6:9] = cmd_obs * cmd_scale
                obs[9 : 9 + num_actions] = qj
                obs[9 + num_actions : 9 + 2 * num_actions] = dqj
                obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
                obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = np.array([sin_phase, cos_phase])
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                # policy inference
                action = policy(obs_tensor).detach().numpy().squeeze()
                # transform action to target_dof_pos
                target_dof_pos = action * action_scale + default_angles

                # Fall: reset the robot upright where the current goal round started,
                # facing the heading you point to (no turn needed after the reset)
                if d.qpos[2] < FALL_HEIGHT:
                    d.qpos[0:2] = obstacles.start
                    d.qpos[2] = m.qpos0[2]
                    d.qpos[3:7] = quat_from_yaw(target_heading)
                    d.qpos[7:] = default_angles
                    d.qvel[:] = 0
                    action = np.zeros(num_actions, dtype=np.float32)
                    target_dof_pos = default_angles.copy()
                    policy.reset_memory()
                    mujoco.mj_forward(m, d)
                    game.show("FALL! reset", d.time)

                # Game: goals, obstacles, trail, camera and scene markers
                robot_xy = d.qpos[0:2].copy()
                robot_heading = get_yaw(quat)
                if game.update(robot_xy, d.time):
                    obstacles.place(robot_xy, game.goal)
                obstacles.update(d, d.time)
                new_hits = obstacles.new_hits(d, d.time)
                if new_hits:
                    hits += new_hits
                    game.score -= new_hits
                    game.show(f"HIT! -{new_hits}", d.time)
                if d.time - last_trail_time >= TRAIL_PERIOD:
                    trail.append(robot_xy)
                    last_trail_time = d.time

                if d.time < game.message_until:
                    hud = game.message
                else:
                    hud = (f"vx {vx:+.2f} m/s  yaw {yaw_cmd:+.2f} rad/s  |  "
                           f"score {game.score}  hits {hits}  time {d.time - game.goal_start:5.1f} s")
                    if game.best_time is not None:
                        hud += f"  best {game.best_time:.1f} s"

                with viewer.lock():
                    draw_scene(viewer.user_scn, game, trail, d.qpos[0:3], robot_heading, target_heading, hud)
                    if FOLLOW_CAMERA:
                        # Chase camera: smoothly turn the view to look along the robot heading
                        azimuth_error = wrap_to_pi(robot_heading - np.radians(viewer.cam.azimuth))
                        viewer.cam.azimuth += np.degrees(CAM_SMOOTHING * azimuth_error)

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                # Sync once per control step: viewer.sync() is too slow to call every physics step.
                viewer.sync()

                # Keep simulated time aligned with wall-clock time.
                time_until_next_step = d.time - (time.time() - start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    print(f"Game over: score {game.score}, hits {hits}" + (f", best time {game.best_time:.1f} s" if game.best_time else ""))
    # Window closed: end the process now (viewer/torch background threads can otherwise keep it alive)
    os._exit(0)
