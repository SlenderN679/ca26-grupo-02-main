import numpy as np

from .settings import GOAL_MAX_DIST, GOAL_MIN_DIST, GOAL_RADIUS, MESSAGE_TIME


class Game:
    """Goals on the floor to walk to: score, timer and a new goal each time one is reached."""

    def __init__(self):
        self.rng = np.random.default_rng()
        self.score = 0
        self.goal = np.zeros(2)
        self.goal_start = 0.0
        self.best_time = None
        self.message = ""
        self.message_until = 0.0

    def new_goal(self, robot_xy, t):
        angle = self.rng.uniform(-np.pi, np.pi)
        dist = self.rng.uniform(GOAL_MIN_DIST, GOAL_MAX_DIST)
        self.goal = robot_xy + dist * np.array([np.cos(angle), np.sin(angle)])
        self.goal_start = t

    def update(self, robot_xy, t):
        """Returns True when the goal was reached (a new goal is placed)."""
        if np.linalg.norm(self.goal - robot_xy) < GOAL_RADIUS:
            goal_time = t - self.goal_start
            self.score += 1
            self.best_time = goal_time if self.best_time is None else min(self.best_time, goal_time)
            self.show(f"GOAL! {goal_time:.1f} s", t)
            self.new_goal(robot_xy, t)
            return True
        return False

    def show(self, message, t):
        self.message = message
        self.message_until = t + MESSAGE_TIME
