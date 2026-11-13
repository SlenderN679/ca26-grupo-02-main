# Game: walk the robot to the goal marker on the floor. Each goal reached scores a point
# and a new goal appears. A chase camera follows the robot, arrows show the robot heading
# (blue) and the target heading (green), and a trail shows the path walked.
# A label above the robot shows the speed command, score and the time on the current goal.
#
# Obstacles: moving solid obstacles are placed between the robot and each new goal:
# a sliding wall (door), a rotating sweeper bar and two pillars circling the goal.
# They really push the robot; each hit costs a point (1 s cooldown per obstacle).
# If the robot falls it is reset upright where the current goal round started.
#
# The robot is commanded with cmd = [vx, vy, yaw_rate]. Without control it stands still.
INITIAL_YAW_DEG = 90.0  # initial robot heading rotation, counter-clockwise seen from above [deg]
SHOW_VIEWER_UI = False  # MuJoCo viewer side panels

# Game
GOAL_MIN_DIST = 4.0     # [m] new goals appear this far from the robot (room for the obstacles)...
GOAL_MAX_DIST = 6.0     # [m] ...up to this far
GOAL_RADIUS = 0.6       # [m] goal reached when the robot base is this close
FOLLOW_CAMERA = True    # chase camera behind the robot
CAM_DISTANCE = 4.0      # [m] camera distance from the robot
CAM_ELEVATION = -20.0   # [deg] camera elevation (negative looks down)
CAM_SMOOTHING = 0.05    # camera azimuth smoothing per control step (0..1, higher follows faster)
ARROW_LENGTH = 0.8      # [m] heading arrows length
TRAIL_PERIOD = 0.25     # [s] one trail dot every period
TRAIL_LENGTH = 200      # trail dots kept
MESSAGE_TIME = 2.0      # [s] "GOAL!" message duration

# Obstacles
WALL_SIZE = [0.1, 0.6, 0.5]         # [m] half sizes (thickness, width, height)
WALL_AT = 0.35                      # wall centre, as a fraction of the robot -> goal path
WALL_AMPLITUDE = 1.0                # [m] sideways slide amplitude
WALL_PERIOD = 4.0                   # [s] slide period
SWEEPER_SIZE = [1.0, 0.05, 0.12]    # [m] half sizes of the bar (half length, thickness, height)
SWEEPER_HEIGHT = 0.3                # [m] bar centre height (shin level)
SWEEPER_AT = 0.65                   # sweeper post, as a fraction of the robot -> goal path
SWEEPER_PERIOD = 6.0                # [s] one turn
PILLAR_RADIUS = 0.25                # [m]
PILLAR_HALF_HEIGHT = 0.6            # [m]
PILLAR_ORBIT = 1.4                  # [m] pillars circle the goal at this radius
PILLAR_PERIOD = 8.0                 # [s] one turn
HIT_COOLDOWN = 1.0                  # [s] one hit per obstacle per cooldown
FALL_HEIGHT = 0.45                  # [m] pelvis below this -> fallen, reset to the round start
