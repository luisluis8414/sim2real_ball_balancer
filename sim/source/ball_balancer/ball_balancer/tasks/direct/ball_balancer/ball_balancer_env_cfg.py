# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import TiledCameraCfg
from gymnasium import spaces
import numpy as np


@configclass
class BallBalancerEnvCfg(DirectRLEnvCfg):
    IMAGE_WIDTH = IMAGE_HEIGHT = 64

    control_joints = ["Revolute_52", "Revolute_57", "Revolute_61"]
    joint_vel_scal = 0.25

    action_scale: float = 0.15
    ball_init_xy_vel_max: float = 0.6
    ball_xy_rand_vel_max: float = 0.6

    # Frequenzen
    decimation = 12
    episode_length_s = 15

    frame_stack_k = 4

    tiled_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/envs/env_.*/Camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.16, 0.0, 0.5),
            rot=(0.6903, 0.1530, 0.1530, 0.6903),
            convention="opengl",
        ),
        data_types=["rgb"],
        # update_period=1,
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=8.0,
            focus_distance=400.0,
            horizontal_aperture=6.4,
            clipping_range=(0.01, 1e7),
        ),
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
    )

    action_space = spaces.Box(
        low=np.float32(-1.0), high=np.float32(1.0), shape=(3,), dtype=np.float32
    )

    observation_space = spaces.Dict(
        {
            "image": spaces.Box(
                low=np.float32(0.0),
                high=np.float32(1.0),
                shape=(12, IMAGE_WIDTH, IMAGE_HEIGHT),
                dtype=np.float32,
            ),
            # stabilzes training a lot and yields far better results. However the real robot only has a camera
            # "joints": spaces.Box(
            #     low=np.float32(-1.0), high=np.float32(1.0), shape=(3,), dtype=np.float32
            # ),
        }
    )

    state_space = spaces.Box(
        low=np.float32(0.0), high=np.float32(0.0), shape=(0,), dtype=np.float32
    )

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=2)

    BALL_BALANCER_CFG = ArticulationCfg(
        prim_path="/World/envs/env_.*/ball_balancer",
        spawn=sim_utils.UsdFileCfg(
            usd_path="../assets/ball_balancer.usd",
            copy_from_source=False,
        ),
        actuators={
            "base": ImplicitActuatorCfg(
                joint_names_expr=control_joints,
                stiffness=10000,
                damping=100,
                velocity_limit=562.0,
            )
        },
    )

    BALL_CFG = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Ball",
        spawn=sim_utils.SphereCfg(
            radius=0.02,  # 4 cm diameter sphere
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=0.5,
                angular_damping=1,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.0027),  # 2.7 g
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.3,  # μs
                dynamic_friction=0.3,  # μd
                restitution=0.87,  # bounciness
                friction_combine_mode="average",
                restitution_combine_mode="average",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.45, 0.0)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.22)),
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=10, env_spacing=1.0, replicate_physics=True
    )
