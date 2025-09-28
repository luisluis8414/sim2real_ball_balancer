# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import TiledCamera
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from .ball_balancer_env_cfg import BallBalancerEnvCfg
import os
from PIL import Image
import numpy as np


class BallBalancerEnv(DirectRLEnv):
    cfg: BallBalancerEnvCfg

    def __init__(
        self, cfg: BallBalancerEnvCfg, render_mode: str | None = None, **kwargs
    ):
        super().__init__(cfg, render_mode, **kwargs)

        self._control_joint_ids: list[int] = []
        for name in self.cfg.control_joints:
            ids, _ = self.robot.find_joints(name)
            self._control_joint_ids.extend(ids)

        self._img_stack = torch.zeros(
            (
                self.num_envs,
                3 * self.cfg.frame_stack_k,
                self.cfg.tiled_camera.height,
                self.cfg.tiled_camera.width,
            ),
            device=self.device,
            dtype=torch.float32,
        )

        self._stack_initialized = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.bool
        )

        self._just_reset = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.bool
        )

        # Random velocity perturbation system
        self._next_perturbation_time = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float32
        )
        self._episode_time = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float32
        )

        self.step_count = 0

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.BALL_BALANCER_CFG)
        self.ball = RigidObject(self.cfg.BALL_CFG)
        self.camera = TiledCamera(self.cfg.tiled_camera)

        spawn_ground_plane("/World/ground", GroundPlaneCfg())

        self.scene.articulations["robot"] = self.robot
        self.scene.rigid_objects["ball"] = self.ball
        self.scene.sensors["camera"] = self.camera

        self.scene.clone_environments(copy_from_source=False)

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._actions = torch.clamp(actions, -1.0, 1.0)

        # Update episode time
        dt = self.cfg.sim.dt * self.cfg.decimation
        self._episode_time += dt

        # Check for random velocity perturbations
        perturbation_mask = self._episode_time >= self._next_perturbation_time
        perturbation_env_ids = torch.where(perturbation_mask)[0]

        if len(perturbation_env_ids) > 0:
            self._apply_random_velocity_perturbation(perturbation_env_ids)
            # Generate new random wait times for these environments
            self._generate_next_perturbation_time(perturbation_env_ids)
            # Update next perturbation time
            self._next_perturbation_time[perturbation_env_ids] = (
                self._episode_time[perturbation_env_ids]
                + self._next_perturbation_time[perturbation_env_ids]
            )

    def _apply_action(self) -> None:
        # joint limits extrahieren
        limits = self.robot.data.joint_pos_limits[
            :, self._control_joint_ids
        ]  # (N, 3, 2)
        low = limits[..., 0]
        high = limits[..., 1]

        current = self.robot.data.joint_pos[:, self._control_joint_ids]  # (N, 3)
        delta = self.cfg.action_scale * self._actions  # (N, 3)
        targets = torch.clamp(current + delta, min=low, max=high)

        self.robot.set_joint_position_target(targets, joint_ids=self._control_joint_ids)
        self.robot.write_data_to_sim()

    def _update_img_stack(self, current_img: torch.Tensor) -> None:
        """
        Aktualisiert den Image Stack für alle Umgebungen.

        Args:
            current_img: Aktuelles Bild (N, 3, H, W)
        """
        K = self.cfg.frame_stack_k

        for env_id in range(self.num_envs):
            # Wenn die Umgebung gerade resetted wurde, überspringen wir das erste Update
            if self._just_reset[env_id]:
                self._just_reset[env_id] = False
                continue

            if not self._stack_initialized[env_id]:
                # Stack ist leer - aktuelles Bild K mal einfügen
                for k in range(K):
                    start_idx = k * 3
                    end_idx = (k + 1) * 3
                    self._img_stack[env_id, start_idx:end_idx] = current_img[env_id]
                self._stack_initialized[env_id] = True
            else:
                # Stack rotieren: alte Frames nach hinten schieben
                # Frame 0 (neuestes) <- aktuelles Bild
                # Frame 1 <- Frame 0 (alt)
                # Frame 2 <- Frame 1 (alt)
                # Frame 3 <- Frame 2 (alt)
                # Frame 3 (ältestes) wird gedroppt

                # Alle Frames um eine Position nach hinten schieben
                for k in range(K - 1, 0, -1):  # von K-1 bis 1 (rückwärts)
                    src_start = (k - 1) * 3
                    src_end = k * 3
                    dst_start = k * 3
                    dst_end = (k + 1) * 3
                    self._img_stack[env_id, dst_start:dst_end] = self._img_stack[
                        env_id, src_start:src_end
                    ]

                # Neues Bild an Position 0 einfügen
                self._img_stack[env_id, 0:3] = current_img[env_id]

    def _get_observations(self) -> dict:
        # (N,H,W,4/3) -> (N,3,H,W) in [0,1]
        img = self.camera.data.output["rgb"][..., :3]
        img = (img.to(torch.float32) / 255.0).permute(0, 3, 1, 2)  # (N,3,H,W)

        # update stack -> self._img_stack: (N, 3*K, H, W)
        self._update_img_stack(img)

        obs_dict = {"image": self._img_stack}

        self._dump_debug_images(self.step_count)

        self.step_count += 1
        return {"policy": obs_dict, "value": obs_dict}

    def _get_rewards(self) -> torch.Tensor:
        # Ball-Absturz
        ball_height = self.ball.data.root_pos_w[:, 2]
        dropped = ball_height < 0.05

        # Relatives XY (Position/Velocity) Ball vs. Plattform
        ball_pos_w = self.ball.data.root_pos_w[:, :3]
        robot_pos_w = self.robot.data.root_pos_w[:, :3]
        rel_ball_pos_xy = ball_pos_w[:, :2] - robot_pos_w[:, :2]

        ball_vel_w = self.ball.data.root_lin_vel_w[:, :3]
        robot_vel_w = self.robot.data.root_lin_vel_w[:, :3]
        rel_ball_vel_xy = ball_vel_w[:, :2] - robot_vel_w[:, :2]

        joint_pos = self.robot.data.joint_pos[:, self._control_joint_ids]  # (N,3)

        return compute_rewards(rel_ball_pos_xy, rel_ball_vel_xy, joint_pos, dropped)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        ball_height = self.ball.data.root_pos_w[:, 2]
        if hasattr(self, "episode_length_buf"):
            dropped = (ball_height < 0.05) & (
                self.episode_length_buf > 1
            )  # vermeidet Reset direkt beim Start
        else:
            dropped = ball_height < 0.05

        time_out = self.episode_length_buf >= self.max_episode_length - 1
        resets = dropped | time_out
        return resets, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = list(range(self.num_envs))

        for env_id in env_ids:
            self._stack_initialized[env_id] = False
            self._img_stack[env_id].zero_()
            self._just_reset[env_id] = True

        self._episode_time[env_ids] = 0.0
        self._generate_next_perturbation_time(env_ids)

        robot_state = self.robot.data.default_root_state.clone()[env_ids]
        robot_state[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_state_to_sim(robot_state, env_ids)

        ball_state = self.ball.data.default_root_state.clone()[env_ids]

        new_xy = (torch.rand(len(env_ids), 2, device=self.device) - 0.5) * 0.10
        ball_state[:, 0:2] = new_xy
        ball_state[:, 2:3] = 0.20

        vmax = self.cfg.ball_init_xy_vel_max
        center_direction = -new_xy
        center_direction_norm = torch.norm(center_direction, dim=1, keepdim=True)
        center_direction = torch.where(
            center_direction_norm > 1e-6,
            center_direction / center_direction_norm,
            torch.tensor([1.0, 0.0], device=self.device).expand(len(env_ids), 2),
        )

        max_angle = math.pi / 6
        random_angles = (
            (torch.rand(len(env_ids), device=self.device) - 0.5) * 2 * max_angle
        )
        cos_angle = torch.cos(random_angles)
        sin_angle = torch.sin(random_angles)
        rotated_x = (
            center_direction[:, 0] * cos_angle - center_direction[:, 1] * sin_angle
        )
        rotated_y = (
            center_direction[:, 0] * sin_angle + center_direction[:, 1] * cos_angle
        )
        velocity_direction = torch.stack([rotated_x, rotated_y], dim=1)

        velocity_magnitude = torch.rand(len(env_ids), device=self.device) * vmax
        ball_state[:, 7:9] = velocity_direction * velocity_magnitude.unsqueeze(1)
        ball_state[:, 9:10] = 0.0
        ball_state[:, 10:13] = 0.0

        ball_state[:, :3] += self.scene.env_origins[env_ids]
        self.ball.write_root_state_to_sim(ball_state, env_ids)

        # Get current joint positions to preserve the correct shape
        current_joint_pos = self.robot.data.joint_pos[env_ids, :].clone()
        current_joint_pos[:, :] = self.robot.data.default_joint_pos[env_ids, :]
        q0_ctrl = current_joint_pos[:, self._control_joint_ids]

        self.robot.data.joint_pos[env_ids, :] = current_joint_pos
        self.robot.data.joint_vel[env_ids, :] = 0.0
        self.robot.write_joint_state_to_sim(
            self.robot.data.joint_pos[env_ids],
            self.robot.data.joint_vel[env_ids],
            None,
            env_ids,
        )

        self.robot.set_joint_position_target(
            q0_ctrl, joint_ids=self._control_joint_ids, env_ids=env_ids
        )
        self.robot.write_data_to_sim()

        self.episode_length_buf[env_ids] = 0

    def _generate_next_perturbation_time(self, env_ids):
        """Generate next random perturbation time between 2-8 seconds"""
        random_wait = (
            torch.rand(len(env_ids), device=self.device) * 6.0 + 2.0
        )  # 2-8 sec
        self._next_perturbation_time[env_ids] = random_wait

    def _apply_random_velocity_perturbation(self, env_ids):
        """Apply random velocity perturbation and spin to specified environments"""
        if len(env_ids) == 0:
            return

        # Generate random direction (unit vector)
        random_angles = torch.rand(len(env_ids), device=self.device) * 2 * math.pi
        direction_x = torch.cos(random_angles)
        direction_y = torch.sin(random_angles)
        direction = torch.stack([direction_x, direction_y], dim=1)

        # Generate random magnitude
        magnitude = (
            torch.rand(len(env_ids), device=self.device) * self.cfg.ball_xy_rand_vel_max
        )

        # Apply velocity perturbation
        velocity_perturbation = direction * magnitude.unsqueeze(1)

        current_vel = self.ball.data.root_lin_vel_w[env_ids, :2]
        new_vel = current_vel + velocity_perturbation

        # Update ball velocity
        full_vel = self.ball.data.root_lin_vel_w[env_ids].clone()
        full_vel[:, :2] = new_vel

        # Add random angular velocity (spin)
        max_spin = 10.0  # rad/s - adjust this value as needed
        random_spin = (
            (torch.rand(len(env_ids), 3, device=self.device) - 0.5) * 2 * max_spin
        )

        current_ang_vel = self.ball.data.root_ang_vel_w[env_ids]
        new_ang_vel = current_ang_vel + random_spin

        # Write velocity and angular velocity to simulation
        self.ball.write_root_link_velocity_to_sim(
            torch.cat([full_vel, new_ang_vel], dim=1), env_ids
        )

    def _dump_debug_images(self, step_count):
        """Speichert die gestackten Frames für env 0 als Grid."""
        env_id = 0
        os.makedirs("debug_images", exist_ok=True)

        if not self._stack_initialized[env_id]:
            return

        K = self.cfg.frame_stack_k
        stacked = self._img_stack[env_id]

        is_first_frame_stack = True
        first_frame = stacked[0:3]

        for k in range(1, K):
            frame_k = stacked[3 * k : 3 * (k + 1)]
            if not torch.allclose(first_frame, frame_k, atol=1e-6):
                is_first_frame_stack = False
                break

        frames = []
        for k in range(K):
            frame_rgb = stacked[3 * k : 3 * (k + 1)].clamp(0, 1) * 255.0
            frame_rgb = frame_rgb.to(torch.uint8).cpu().numpy()
            frame_rgb = np.moveaxis(frame_rgb, 0, 2)  # (H, W, 3)
            frames.append(frame_rgb)

        grid = np.concatenate(frames, axis=1)  # (H, W*K, 3)

        img_pil = Image.fromarray(grid)

        if is_first_frame_stack:
            filename = f"first_frame_stack_{step_count:06d}.png"
        else:
            filename = f"step_{step_count:06d}_stack.png"

        img_pil.save(os.path.join("debug_images", filename))

    def _dump_single_images(self, step_count):
        """Speichert die einzelnen Frames für env 0 in separate Dateien."""
        env_id = 0
        os.makedirs("single_images", exist_ok=True)

        if not self._stack_initialized[env_id]:
            return

        K = self.cfg.frame_stack_k
        stacked = self._img_stack[env_id]

        # Check if all frames are identical (first frame stack)
        is_first_frame_stack = True
        first_frame = stacked[0:3]

        for k in range(1, K):
            frame_k = stacked[3 * k : 3 * (k + 1)]
            if not torch.allclose(first_frame, frame_k, atol=1e-6):
                is_first_frame_stack = False
                break

        # Save each frame individually
        for k in range(K):
            frame_rgb = stacked[3 * k : 3 * (k + 1)].clamp(0, 1) * 255.0
            frame_rgb = frame_rgb.to(torch.uint8).cpu().numpy()
            frame_rgb = np.moveaxis(frame_rgb, 0, 2)  # (H, W, 3)

            img_pil = Image.fromarray(frame_rgb)

            if is_first_frame_stack:
                filename = f"first_frame_stack_{step_count:06d}_frame_{k}.png"
            else:
                filename = f"step_{step_count:06d}_frame_{k}.png"

            img_pil.save(os.path.join("single_images", filename))


@torch.jit.script
def compute_rewards(
    rel_ball_pos_xy: torch.Tensor,
    rel_ball_vel_xy: torch.Tensor,
    joint_pos: torch.Tensor,
    dropped: torch.Tensor,
) -> torch.Tensor:
    # Existing calculations
    pos_cost = torch.sum(rel_ball_pos_xy * rel_ball_pos_xy, dim=-1)  # m^2
    vel_cost = torch.sum(rel_ball_vel_xy * rel_ball_vel_xy, dim=-1)  # (m/s)^2

    # Calculate stability metrics
    dist = torch.sqrt(pos_cost + 1e-8)
    speed = torch.sqrt(vel_cost + 1e-8)

    # Dynamic joint penalty based on ball stability
    stability_radius = 0.02  # 2 cm
    stability_speed = 0.05  # 5 cm/s

    is_stable = (dist < stability_radius) & (speed < stability_speed)

    # Higher joint penalty when ball is stable
    base_joint_penalty = 0.02
    stable_joint_penalty = 0.1  # 5x higher penalty when stable

    joint_penalty = torch.where(is_stable, stable_joint_penalty, base_joint_penalty)

    tilt_cost = torch.sum(joint_pos * joint_pos, dim=-1) * joint_penalty

    # Rest of reward calculation
    center_radius = 0.01
    center_bonus = torch.clamp(center_radius - dist, min=0.0) / center_radius

    reward = 1.0 + 0.5 * center_bonus - 10.0 * pos_cost - 1.0 * vel_cost - tilt_cost

    drop_penalty = 20.0
    reward = torch.where(dropped, -drop_penalty * torch.ones_like(reward), reward)
    return reward
