"""
ApexVelocity Gymnasium Environment.

⚠️ Experimental:
    This RL environment is a simplified demo. It uses synthetic paths and a
    lightweight internal physics model and does NOT yet integrate with
    `OSMLoader`, real OSM routes, or the C++ core solver.

    API and behavior may change in future 1.x releases. Treat this as an
    example for how one might build a Gymnasium environment on top of
    ApexVelocity, not as a production-ready AV environment.

A reinforcement learning environment for vehicle speed control
using the ApexVelocity physics engine.
"""

from typing import Optional, Tuple, Dict, Any, List
import math

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
    _HAS_GYM = True
except ImportError:
    _HAS_GYM = False
    gym = None
    spaces = None


class ApexVelocityEnv:
    """
    Gymnasium environment for vehicle speed control.
    
    The agent controls throttle/brake to navigate a path while:
    - Respecting physics-based speed limits (curvature, friction)
    - Minimizing energy consumption
    - Maximizing progress along the path
    
    Observation Space:
        - current_speed: Current vehicle speed (m/s)
        - target_speed: Physics-limited target speed (m/s)
        - curvature_lookahead: Array of upcoming curvatures
        - surface_friction: Current surface friction coefficient
        - distance_remaining: Distance to goal (m)
        - energy_used: Cumulative energy used (normalized)
    
    Action Space:
        - Continuous [-1, 1]: -1 = full brake, +1 = full throttle
    
    Rewards:
        - Progress reward: positive for moving forward
        - Energy penalty: negative for energy consumption
        - Safety penalty: negative for exceeding speed limits
        - Goal bonus: positive for reaching the end
    """
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(
        self,
        path_length: float = 1000.0,
        lookahead_points: int = 10,
        lookahead_distance: float = 100.0,
        dt: float = 0.1,
        vehicle: str = "default",
        surface: str = "asphalt",
        condition: str = "dry",
        max_episode_steps: int = 1000,
        render_mode: Optional[str] = None,
    ):
        """
        Initialize the environment.
        
        Args:
            path_length: Total path length in meters
            lookahead_points: Number of lookahead curvature points
            lookahead_distance: Distance to look ahead (m)
            dt: Simulation timestep (s)
            vehicle: Vehicle preset name
            surface: Default surface type
            condition: Surface condition ("dry" or "wet")
            max_episode_steps: Maximum steps per episode
            render_mode: Render mode ("human" or "rgb_array")
        """
        if not _HAS_GYM:
            raise ImportError("gymnasium is required for ApexVelocityEnv")
        
        self.path_length = path_length
        self.lookahead_points = lookahead_points
        self.lookahead_distance = lookahead_distance
        self.dt = dt
        self.vehicle_name = vehicle
        self.surface = surface
        self.condition = condition
        self.max_episode_steps = max_episode_steps
        self.render_mode = render_mode
        
        # Vehicle parameters (simplified for stub)
        self.mass = 1500.0  # kg
        self.max_accel = 3.0  # m/s^2
        self.max_brake = 8.0  # m/s^2
        self.max_speed = 50.0  # m/s
        self.drag_coeff = 0.3
        self.frontal_area = 2.2
        self.air_density = 1.225
        
        # Friction coefficient
        self.mu = 0.9 if condition == "dry" else 0.6
        
        # Observation space
        # [speed, target_speed, curvature_lookahead..., friction, distance_remaining, energy_normalized]
        obs_dim = 4 + lookahead_points
        self.observation_space = spaces.Box(
            low=np.array([0, 0] + [-1.0] * lookahead_points + [0, 0, 0], dtype=np.float32),
            high=np.array([100, 100] + [1.0] * lookahead_points + [1, path_length, 1], dtype=np.float32),
            dtype=np.float32
        )
        
        # Action space: continuous throttle/brake
        self.action_space = spaces.Box(
            low=np.array([-1.0], dtype=np.float32),
            high=np.array([1.0], dtype=np.float32),
            dtype=np.float32
        )
        
        # State
        self.position = 0.0
        self.speed = 0.0
        self.energy_used = 0.0
        self.step_count = 0
        
        # Pre-generate path with random curvatures
        self._generate_path()
    
    def _generate_path(self):
        """Generate a random path with varying curvatures."""
        np.random.seed()
        
        # Sample points along the path
        num_segments = 20
        segment_length = self.path_length / num_segments
        
        self.path_curvatures = []
        self.path_positions = []
        
        for i in range(num_segments + 1):
            pos = i * segment_length
            # Random curvature (some straight sections, some curves)
            if np.random.random() < 0.3:
                curvature = 0.0  # Straight
            else:
                # Random curve with radius 20-200m
                radius = np.random.uniform(20, 200)
                curvature = 1.0 / radius * np.random.choice([-1, 1])
            
            self.path_positions.append(pos)
            self.path_curvatures.append(curvature)
    
    def _get_curvature_at(self, position: float) -> float:
        """Get interpolated curvature at a position."""
        if position < 0:
            return self.path_curvatures[0]
        if position >= self.path_length:
            return self.path_curvatures[-1]
        
        # Find segment
        for i in range(len(self.path_positions) - 1):
            if self.path_positions[i] <= position < self.path_positions[i + 1]:
                # Linear interpolation
                t = (position - self.path_positions[i]) / (
                    self.path_positions[i + 1] - self.path_positions[i]
                )
                return (1 - t) * self.path_curvatures[i] + t * self.path_curvatures[i + 1]
        
        return 0.0
    
    def _get_target_speed(self, curvature: float) -> float:
        """Calculate physics-limited target speed for a curvature."""
        if abs(curvature) < 1e-6:
            return self.max_speed
        
        # v_max = sqrt(mu * g / kappa)
        g = 9.81
        v_max = math.sqrt(self.mu * g / abs(curvature))
        return min(v_max, self.max_speed)
    
    def _get_observation(self) -> np.ndarray:
        """Build observation array."""
        # Current curvature and target speed
        current_curv = self._get_curvature_at(self.position)
        target_speed = self._get_target_speed(current_curv)
        
        # Lookahead curvatures
        lookahead = []
        for i in range(self.lookahead_points):
            dist = self.position + (i + 1) * (self.lookahead_distance / self.lookahead_points)
            curv = self._get_curvature_at(dist)
            # Normalize curvature to roughly [-1, 1]
            lookahead.append(np.clip(curv * 100, -1, 1))
        
        # Distance remaining (normalized)
        dist_remaining = max(0, self.path_length - self.position)
        
        # Energy normalized (rough estimate)
        max_energy = self.mass * self.max_speed**2  # Very rough upper bound
        energy_norm = min(self.energy_used / max_energy, 1.0)
        
        obs = np.array(
            [self.speed, target_speed] + lookahead + [self.mu, dist_remaining, energy_norm],
            dtype=np.float32
        )
        
        return obs
    
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset the environment."""
        if seed is not None:
            np.random.seed(seed)
        
        # Reset state
        self.position = 0.0
        self.speed = 0.0
        self.energy_used = 0.0
        self.step_count = 0
        
        # Regenerate path
        self._generate_path()
        
        obs = self._get_observation()
        info = {"position": self.position, "speed": self.speed}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Execute one step.
        
        Args:
            action: Throttle/brake command [-1, 1]
        
        Returns:
            observation, reward, terminated, truncated, info
        """
        self.step_count += 1
        
        # Extract action
        throttle_brake = float(np.clip(action[0], -1, 1))
        
        # Calculate acceleration
        if throttle_brake >= 0:
            accel = throttle_brake * self.max_accel
        else:
            accel = throttle_brake * self.max_brake
        
        # Add drag
        drag_force = 0.5 * self.air_density * self.drag_coeff * self.frontal_area * self.speed**2
        drag_accel = drag_force / self.mass
        accel -= drag_accel
        
        # Update speed
        old_speed = self.speed
        self.speed = max(0, self.speed + accel * self.dt)
        
        # Update position
        avg_speed = (old_speed + self.speed) / 2
        distance = avg_speed * self.dt
        self.position += distance
        
        # Calculate energy (simplified: F * d)
        if throttle_brake > 0:
            force = throttle_brake * self.max_accel * self.mass + drag_force
            energy = force * distance
            self.energy_used += max(0, energy)
        
        # Get target speed for current position
        current_curv = self._get_curvature_at(self.position)
        target_speed = self._get_target_speed(current_curv)
        
        # Calculate reward
        reward = 0.0
        
        # Progress reward
        reward += distance * 0.01
        
        # Energy penalty
        if throttle_brake > 0:
            reward -= energy * 1e-6
        
        # Safety penalty for exceeding target speed
        if self.speed > target_speed * 1.1:
            overspeed = self.speed - target_speed
            reward -= overspeed * 0.1
        
        # Check termination
        terminated = self.position >= self.path_length
        truncated = self.step_count >= self.max_episode_steps
        
        # Goal bonus
        if terminated:
            reward += 100.0
        
        obs = self._get_observation()
        info = {
            "position": self.position,
            "speed": self.speed,
            "target_speed": target_speed,
            "energy_used": self.energy_used,
            "curvature": current_curv,
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        """Render the environment (stub)."""
        if self.render_mode == "human":
            print(f"Position: {self.position:.1f}m, Speed: {self.speed:.1f}m/s, "
                  f"Energy: {self.energy_used/1000:.1f}kJ")
        return None
    
    def close(self):
        """Clean up resources."""
        pass


# Register with Gymnasium if available
if _HAS_GYM:
    try:
        gym.register(
            id="ApexVelocity-v0",
            entry_point="apexvelocity.envs:ApexVelocityEnv",
            max_episode_steps=1000,
        )
    except Exception:
        pass  # May already be registered





