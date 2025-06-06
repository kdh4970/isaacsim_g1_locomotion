# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional

import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
try:
    from .controllers.policy_controller import PolicyController
except ImportError:
    from controllers.policy_controller import PolicyController
from isaacsim.storage.native import get_assets_root_path
from pathlib import Path
import os 

class G1FlatTerrainPolicy(PolicyController):
    """The G1 Humanoid running Flat Terrain Policy Locomotion Policy"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "g1",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize G1 robot and import flat terrain policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """
        assets_root_path = get_assets_root_path()
        print(assets_root_path)
        if usd_path == None:
            # usd_path = assets_root_path + "/Isaac/IsaacLab/Robots/Unitree/G1/g1_minimal.usd"
            usd_path = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/g1.usd"
            # usd_path = assets_root_path + "/Isaac/Robots/Unitree/G1/G1_with_hand/g1_29dof_with_hand_rev_1_0.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        cwd = str(Path(__file__).resolve().parent)
        policy_path = cwd + "/G1_Policies/policy.pt"
        env_path = cwd + "/G1_Policies/g1_env.yaml"
        if not os.path.exists(policy_path):
            raise FileNotFoundError(f"Could not find Policy file : {policy_path}")
        if not os.path.exists(env_path):
            raise FileNotFoundError(f"Could not find Env file : {env_path}")

        self.load_policy(policy_path, env_path)

        self._action_scale = 0.5
        self._previous_action = np.zeros(37)
        self._policy_counter = 0

    def _compute_observation(self, command):
        """
        Compute the observation vector for the policy.

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(123)
        # Base lin vel
        obs[:3] = lin_vel_b
        # Base ang vel
        obs[3:6] = ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9:12] = command
        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:49] = current_joint_pos - self.default_pos
        obs[49:86] = current_joint_vel
        # Previous Action
        obs[86:123] = self._previous_action
        return obs

    def forward(self, dt, command):
        """
        Compute the desired articulation action and apply them to the robot articulation.

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self):
        """
        Overloads the default initialize function to use default articulation root properties in the USD
        """
        return super().initialize(set_articulation_props=False)
