"""Gymnasium task with geometric collision, A* route and moving local goal."""
import copy
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from .geometry import raycast, clearance_at, plan_path, integrate
from .observation import make_observation
from .control import scale_action, limit_rate, filter_velocity
from .goal import select_local_goal

STAGE_NAMES = ('empty', 'single', 'turn', 'clutter', 'dense', 'tight_turn', 'protective')


class IndoorNavEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, cfg, stage=None, safety=None):
        super().__init__()
        self.cfg = copy.deepcopy(cfg)
        if stage:
            self.cfg['environment']['stage'] = stage
        if safety is not None:
            self.cfg['safety']['enabled'] = safety
        n = self.cfg['observation']['size']
        low = np.concatenate([np.zeros(2 * n * n), np.array([-1, -1, 0, -1, 0, -1, 0, -1])]).astype(np.float32)
        high = np.ones_like(low)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, (2,), dtype=np.float32)

    def _choose_stage(self):
        configured = self.cfg['environment']['stage']
        if configured != 'mixed':
            return configured
        weights = self.cfg['environment']['stage_weights']
        names = list(weights)
        probabilities = np.asarray([weights[name] for name in names], dtype=float)
        probabilities = probabilities / probabilities.sum()
        return str(self.np_random.choice(names, p=probabilities))

    def _make_layout(self, stage):
        half = self.cfg['environment']['room_half_size']
        rectangles = [
            [-half - 0.2, -half - 0.2, -half, half + 0.2],
            [half, -half - 0.2, half + 0.2, half + 0.2],
            [-half, -half - 0.2, half, -half],
            [-half, half, half, half + 0.2],
        ]
        if stage == 'turn':
            rectangles.append([-0.2, -1.8, 0.2, 0.7])
        elif stage == 'tight_turn':
            rectangles.append([-0.2, -1.8, 0.2, 0.7])
            rectangles.append([0.45, 1.55, 1.05, 2.05])
        elif stage == 'protective':
            # Narrow 0.8 m corridor; start and goal are sampled inside it.
            rectangles.append([-half - 0.2, -half - 0.2, -0.4, half + 0.2])
            rectangles.append([0.4, -half - 0.2, half + 0.2, half + 0.2])
        else:
            count = {'empty': 0, 'single': 1, 'clutter': 5, 'dense': 9}[stage]
            width_range = (0.20, 0.45) if stage == 'dense' else (0.25, 0.65)
            extent = 2.2 if stage == 'dense' else 1.8
            for _ in range(count):
                center = self.np_random.uniform(-extent, extent, 2)
                width = self.np_random.uniform(*width_range, 2)
                rectangles.append([*(center - width / 2), *(center + width / 2)])
        return np.asarray(rectangles, dtype=float)

    def _free_cells(self):
        environment = self.cfg['environment']
        robot = self.cfg['robot']
        margin = environment['planner_margin'] + 0.02
        half = environment['room_half_size'] - robot['radius'] - 0.3
        resolution = environment['task_grid_resolution']
        axis = np.arange(-half, half + 1e-9, resolution)
        x, y = np.meshgrid(axis, axis)
        clearance = np.full_like(x, np.inf)
        for rectangle in self.rectangles:
            dx = np.maximum(np.maximum(rectangle[0] - x, 0), x - rectangle[2])
            dy = np.maximum(np.maximum(rectangle[1] - y, 0), y - rectangle[3])
            clearance = np.minimum(clearance, np.hypot(dx, dy) - robot['radius'])
        free = clearance >= margin
        return np.column_stack([x[free], y[free]])

    def _sample_task(self, stage):
        environment = self.cfg['environment']
        for _ in range(environment['layout_attempts']):
            self.rectangles = self._make_layout(stage)
            cells = self._free_cells()
            if len(cells) < 2:
                continue
            for _ in range(environment['task_attempts']):
                indices = self.np_random.integers(0, len(cells), size=2)
                start = cells[indices[0]]
                goal = cells[indices[1]]
                distance = float(np.linalg.norm(goal - start))
                if distance < environment['min_goal_distance'] or distance > environment['max_goal_distance']:
                    continue
                path = plan_path(start, goal, self.rectangles, self.cfg)
                if path is not None:
                    return start, goal, path
        raise RuntimeError(f'Could not generate reachable task for fixed stage {stage}')

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        environment = self.cfg['environment']
        robot = self.cfg['robot']
        stage = self._choose_stage()
        start, goal, path = self._sample_task(stage)
        self.actual_stage = stage
        self.goal = goal
        self.path = path
        self.arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
        self.reference_length = float(self.arc[-1])
        self.pose = np.array([*start, self.np_random.uniform(-math.pi, math.pi)])
        self.velocity = np.zeros(2)
        self.previous_executed = np.zeros(2)
        self.steps = 0
        self.progress = 0.0
        self.path_index = 0
        self.hold = 0.0
        self.trajectory = [self.pose.copy()]
        self.path_length = 0.0
        self.interventions = 0
        self.intervention_steps = 0
        self.intervention_counts = {}
        self.was_intervening = False
        self.requested_distance = 0.0
        self.executed_distance = 0.0
        self.reward_sums = {}
        self.no_progress_time = 0.0
        self.turn_grace_used = 0.0
        self.min_clearance = clearance_at(self.pose, self.rectangles, robot['radius'])
        self.max_steps = max(environment['max_steps'], int(math.ceil(
            (environment['timeout_factor'] * self.reference_length / robot['v_max'] + 15) / robot['dt'])))
        self.ranges = self._scan()
        self.path_index = 0
        _, self.local_goal = select_local_goal(self.path, self.arc, self.pose, self.cfg, 0)
        self.previous_heading_error = self._heading_error()
        return self._observation(), {'stage': self.actual_stage, 'reference_length': self.reference_length}

    def _scan(self):
        sensor = self.cfg['sensor']
        ranges = raycast(self.pose, self.rectangles, sensor['beams'], sensor['range_min'],
                         sensor['range_max'], sensor['x'], sensor['y'], sensor['yaw'])
        if sensor['noise_std']:
            valid = np.isfinite(ranges)
            ranges[valid] += self.np_random.normal(0, sensor['noise_std'], np.count_nonzero(valid)).astype(np.float32)
            ranges[valid] = np.clip(ranges[valid], sensor['range_min'], sensor['range_max'])
        return ranges

    def _heading_error(self):
        delta = self.local_goal - self.pose[:2]
        cosine, sine = math.cos(self.pose[2]), math.sin(self.pose[2])
        local_x = cosine * delta[0] + sine * delta[1]
        local_y = -sine * delta[0] + cosine * delta[1]
        return math.atan2(local_y, local_x)

    def _update_route(self):
        """Bounded nearest-point search prevents jumps at crossing/folded routes."""
        low = max(0, self.path_index - 4)
        high = min(len(self.path), self.path_index + 35)
        self.path_index = low + int(np.argmin(np.linalg.norm(self.path[low:high] - self.pose[:2], axis=1)))
        self.progress = float(self.arc[self.path_index])
        _, self.local_goal = select_local_goal(
            self.path, self.arc, self.pose, self.cfg, self.path_index)

    def _observation(self):
        delta = self.local_goal - self.pose[:2]
        cosine, sine = math.cos(self.pose[2]), math.sin(self.pose[2])
        local = np.array([cosine * delta[0] + sine * delta[1], -sine * delta[0] + cosine * delta[1]])
        return make_observation(self.ranges, local, self.velocity, self.previous_executed, self.cfg)

    def _compute_reward(self, delta_progress, clearance, success, collision, old_w,
                        requested, executed, intervening):
        """Primary tuning point: every component is exposed in episode summaries."""
        reward = self.cfg['reward']
        scale = np.array([self.cfg['robot']['v_max'], self.cfg['robot']['w_max']])
        deviation = np.linalg.norm((np.asarray(requested) - np.asarray(executed)) / scale)
        terms = {
            'progress': reward['progress'] * delta_progress,
            'time': reward['time'],
            'clearance': reward['clearance'] * max(
                0, (reward['clearance_threshold'] - clearance) / reward['clearance_threshold']),
            'smoothness': reward['smoothness'] * abs(self.velocity[1] - old_w),
            'success': reward['success'] if success else 0.0,
            'collision': reward['collision'] if collision else 0.0,
            'intervention': reward.get('intervention', 0.0) * float(intervening),
            'action_deviation': reward.get('action_deviation', 0.0) * float(deviation),
        }
        for key, value in terms.items():
            self.reward_sums[key] = self.reward_sums.get(key, 0.0) + float(value)
        return float(sum(terms.values())), terms

    def step(self, action):
        robot = self.cfg['robot']
        environment = self.cfg['environment']
        raw_action = np.asarray(action, dtype=np.float32).copy()
        old_pose = self.pose.copy()
        old_progress = self.progress
        old_velocity = self.velocity.copy()
        target = scale_action(raw_action, self.cfg)
        rate_limited = limit_rate(target, self.velocity, self.cfg)
        self.velocity, intervening, intervention_reason = filter_velocity(
            rate_limited, self.ranges, self.cfg)
        self.interventions += int(intervening and not self.was_intervening)
        self.intervention_steps += int(intervening)
        if intervening:
            self.intervention_counts[intervention_reason] = self.intervention_counts.get(intervention_reason, 0) + 1
        self.was_intervening = intervening
        self.requested_distance += float(np.linalg.norm(rate_limited)) * robot['dt']
        self.executed_distance += float(np.linalg.norm(self.velocity)) * robot['dt']
        collision = False
        for _ in range(4):
            self.pose = integrate(self.pose, *self.velocity, robot['dt'] / 4)
            clearance = clearance_at(self.pose, self.rectangles, robot['radius'])
            self.min_clearance = min(self.min_clearance, clearance)
            if clearance <= 0:
                collision = True
                break
        self.previous_executed = self.velocity.copy()
        self.steps += 1
        self.path_length += float(np.linalg.norm(self.pose[:2] - old_pose[:2]))
        self.trajectory.append(self.pose.copy())
        self._update_route()
        delta_progress = self.progress - old_progress
        within = np.linalg.norm(self.goal - self.pose[:2]) <= environment['goal_radius']
        self.hold = self.hold + robot['dt'] if within else 0.0
        success = self.hold >= environment['goal_hold_seconds'] - 1e-8 and not collision

        if delta_progress < environment['progress_epsilon']:
            self.no_progress_time += robot['dt']
        else:
            self.no_progress_time = 0.0
            self.turn_grace_used = 0.0
        heading_error = self._heading_error()
        turning_toward_goal = (
            abs(self.velocity[1]) >= environment['turn_rate_threshold']
            and abs(heading_error) < abs(self.previous_heading_error) - 1e-3
        )
        if self.no_progress_time > 0 and turning_toward_goal:
            self.turn_grace_used = min(
                environment['turn_grace_seconds'], self.turn_grace_used + robot['dt'])
        self.previous_heading_error = heading_error
        stuck = self.no_progress_time >= environment['stuck_seconds'] + self.turn_grace_used
        terminated = bool(collision or success or stuck)
        truncated = bool(self.steps >= self.max_steps and not terminated)
        if collision:
            reason = 'collision'
        elif success:
            reason = 'success'
        elif stuck:
            reason = 'stuck'
        elif truncated:
            reason = 'timeout'
        else:
            reason = ''
        reward, terms = self._compute_reward(
            delta_progress, clearance, success, collision, old_velocity[1],
            rate_limited, self.velocity, intervening)
        self.ranges = self._scan()
        info = {
            'reward_terms': terms,
            'success': bool(success),
            'collision': bool(collision),
            'reason': reason,
            'intervening': bool(intervening),
            'intervention_reason': intervention_reason,
            'raw_action': raw_action.tolist(),
            'requested_velocity': rate_limited.tolist(),
            'executed_velocity': self.velocity.tolist(),
        }
        if terminated or truncated:
            duration = self.steps * robot['dt']
            info['summary'] = {
                'success': bool(success),
                'collision': bool(collision),
                'reason': reason,
                'stage': self.actual_stage,
                'steps': self.steps,
                'sim_time': duration,
                'path_length': self.path_length,
                'reference_length': self.reference_length,
                'min_clearance': self.min_clearance,
                'interventions': self.interventions,
                'intervention_duration': self.intervention_steps * robot['dt'],
                'intervention_reasons': dict(self.intervention_counts),
                'mean_requested_speed': self.requested_distance / max(duration, 1e-9),
                'mean_executed_speed': self.executed_distance / max(duration, 1e-9),
                'no_progress_time': self.no_progress_time,
                'turn_grace_used': self.turn_grace_used,
                **{'reward_' + key: value for key, value in self.reward_sums.items()},
            }
        return self._observation(), reward, terminated, truncated, info
