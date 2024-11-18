import numpy as np

import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
import time

from utils import dict_to_list, list_to_list


class CommandGenerator:
    def __init__(self, init_cmd: list = [0, 0, 0]) -> None:
        # [vy, vy, yaw_rate]
        self.vel_cmd = init_cmd

    def __call__(self) -> list:
        return self.vel_cmd

    def set_vel_cmd(self, vel_cmd: list) -> None:
        self.vel_cmd = vel_cmd


class Env:
    def __init__(self, cfg: dict) -> None:
        # Get PyBullet client
        self.client = bc.BulletClient(connection_mode=p.GUI)
        self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Initiate simulation
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(enableConeFriction=0)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set up simulation
        self.sim_f = cfg["sim_f"]
        self.sim_dt = 1. / self.sim_f
        self.control_f = cfg["control_f"]
        self.control_dt = 1. / self.control_f
        self.repeat = int(self.sim_f / self.control_f)
        self.client.setTimeStep(self.sim_dt)
        self.gravity = [0, 0, -9.81]
        self.client.setGravity(*self.gravity)

        # Set up command generator
        self.command_generator = CommandGenerator(cfg["command"])

        # Set up ground
        self.ground = self.client.loadURDF("plane.urdf")
        # self.client.changeDynamics(
        #     self.ground, -1,
        #     lateralFriction=2, rollingFriction=2
        # )

        # Set up robot
        # self.Kp, self.Kd, self.Ka = cfg["Kp"], cfg["Kd"], cfg["Ka"]
        self.vel_lim = cfg["vel_lim"]
        self.saturation_lim = cfg["saturation_lim"]
        self.tau_lim = cfg["tau_lim"]
        init_pose = [0.0, 0.0, 0.5]
        init_ori = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        flags = p.URDF_MERGE_FIXED_LINKS  # \
        #     | p.URDF_USE_SELF_COLLISION  # \
        #   | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.robot = self.client.loadURDF(
            "robots/spot/spot.urdf", init_pose, init_ori, flags=flags
        )
        self.n = self.client.getNumJoints(self.robot)
        self.joints = {}
        self.links = {}
        for j in range(self.n):
            self.client.setJointMotorControl2(
                self.robot, j, p.VELOCITY_CONTROL,
                force=0
            )
            self.client.enableJointForceTorqueSensor(
                self.robot, j, 1
            )
            # self.client.changeDynamics(
            #     self.robot, j, linearDamping=0, angularDamping=0,
            #     lateralFriction=cfg["lateral_friction"],
            #     rollingFriction=cfg["rolling_friction"],
            # )
            info = self.client.getJointInfo(self.robot, j)
            joint_name = info[1].decode("utf8")
            joint_type = info[2]
            if (joint_type in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                self.joints[joint_name] = j
                self.links[joint_name.split("_joint")[0]] = j
        info = self.client.getDynamicsInfo(self.robot, -1)
        self.client.changeDynamics(
            self.robot, -1, linearDamping=0, angularDamping=0,
            lateralFriction=cfg["lateral_friction"],
            rollingFriction=cfg["rolling_friction"],
        )
        print(self.joints)
        # assert False
        self.links["trunk"] = -1
        self.joints_isaac = cfg["isaac_joint_order"]
        self.q_init = cfg["q_stance"]
        #print(self.q_init.keys())
        #print(self.joints.keys())
        self.q_init_arr = np.array(dict_to_list(self.q_init, self.joints))
        self.dq_init_arr = np.array([0 for _ in range(self.n)])
        self.tau_lim_arr = np.array([self.tau_lim for _ in range(self.n)])
        self.Kp = np.array(dict_to_list(cfg["Kp_joints"], self.joints))
        self.Kd = np.array(dict_to_list(cfg["Kd_joints"], self.joints))
        self.Ka = cfg["Ka"]
        # self.Kp_arr = [self.Kp] * self.n
        # self.Kd_arr = [self.Kd] * self.n
        self.Kp_arr = self.Kp.tolist()
        self.Kd_arr = self.Kd.tolist()
        self.action_init = [0 for _ in range(self.n)]

        for j in range(self.n):
            self.client.resetJointState(
                self.robot, j, self.q_init_arr[j]
            )

    def step_init(self) -> list:
        mean_torque = np.zeros(self.n)
        for _ in range(self.repeat):
            # PD Mode
            """self.client.setJointMotorControlArray(
                self.robot,
                [j for j in range(self.n)],
                p.POSITION_CONTROL,
                targetPositions=self.q_init_arr,
                targetVelocities=self.dq_init_arr,
                forces=self.tau_lim_arr,
                positionGains=self.Kp_arr,
                velocityGains=self.Kd_arr,
            )"""
            # Torque Mode
            states = self.client.getJointStates(
                self.robot,
                [j for j in range(self.n)]
            )
            q, dq = [], []
            for state in states:
                q.append(state[0])
                dq.append(state[1])
            q_des = self.q_init_arr
            q_err = q_des - np.array(q)
            dq_err = - np.array(dq)
            torque = self.saturate(
                self.Kp * q_err + self.Kd * dq_err,
                np.array(dq)
            )
            self.client.setJointMotorControlArray(
                self.robot,
                [j for j in range(self.n)],
                p.TORQUE_CONTROL,
                forces=torque
            )
            mean_torque += torque
            self.client.stepSimulation()
            time.sleep(self.sim_dt)
        self.action = self.action_init
        obs = self.get_obs()
        self.applied_torque = (mean_torque / self.repeat).tolist()
        self.applied_torque_isaac = list_to_list(
            self.applied_torque,
            self.joints,
            self.joints_isaac
        )
        q_des_list = list_to_list(q_des.tolist(), self.joints, self.joints_isaac)
        return obs, self.applied_torque_isaac, q_des_list

    def step(self, action) -> list:
        # Convert Isaac to PyBullet Ordering
        action = np.array(list_to_list(action, self.joints_isaac, self.joints))
        mean_torque = np.zeros(self.n)
        for _ in range(self.repeat):
            # PD Mode
            """self.client.setJointMotorControlArray(
                self.robot,
                [j for j in range(self.n)],
                p.POSITION_CONTROL,
                targetPositions=self.q_init_arr + self.Ka * action,
                targetVelocities=self.dq_init_arr,
                forces=self.tau_lim_arr,
                positionGains=self.Kp_arr,
                velocityGains=self.Kd_arr,
            )"""
            # Torque Mode
            states = self.client.getJointStates(
                self.robot,
                [j for j in range(self.n)]
            )
            q, dq = [], []
            for state in states:
                q.append(state[0])
                dq.append(state[1])
            q_des = (self.q_init_arr + self.Ka * action)
            q_err = q_des - np.array(q)
            dq_err = - np.array(dq)  # zero desired dq
            torque = self.saturate(
                self.Kp * q_err + self.Kd * dq_err,
                np.array(dq)
            )
            self.client.setJointMotorControlArray(
                self.robot,
                [j for j in range(self.n)],
                p.TORQUE_CONTROL,
                forces=torque
            )
            mean_torque += torque
            #print("position error: ", q_err)
            # print("velocity error: ", dq_err)
            # print("torque: ", torque)
            # print()
            self.client.stepSimulation()
            time.sleep(self.sim_dt)
        self.action = action.tolist()
        obs = self.get_obs()
        self.applied_torque = (mean_torque / self.repeat).tolist()
        self.applied_torque_isaac = list_to_list(
            self.applied_torque,
            self.joints,
            self.joints_isaac
        )
        q_des_list = list_to_list(q_des.tolist(), self.joints, self.joints_isaac)
        return obs, self.applied_torque_isaac, q_des_list

    def quat_rot_inv(self, quat_list: list, gravity: list) -> list:
        quat = np.array(quat_list)
        gravity = np.array(gravity)
        q_w = quat[3]
        q_vec = quat[:3]
        a = gravity * (2.0 * q_w**2 - 1.0)
        b = np.cross(q_vec, gravity) * q_w * 2.0
        c = q_vec * q_vec * gravity
        return (a - b + c).tolist()

    def saturate(self, torque: np.ndarray, dq: np.ndarray) -> np.ndarray:
        max_effort = self.saturation_lim * (1 - dq / self.vel_lim)
        max_effort = np.clip(max_effort, 0, self.tau_lim)
        min_effort = self.saturation_lim * (-1 - dq / self.vel_lim)
        min_effort = np.clip(min_effort, -self.tau_lim, 0)
        return np.clip(torque, min_effort, max_effort)
        # return torque

    def get_obs(self) -> list:

        obs = []

        # Linear and Angular Velocity (Add Missing Terms)
        vel_lin, vel_ang = self.client.getBaseVelocity(self.robot)
        obs += vel_lin  # Linear velocity (3 terms)
        obs += vel_ang  # Angular velocity (3 terms)


        # Projected gravity
        _, ori = self.client.getBasePositionAndOrientation(self.robot)
        obs += self.quat_rot_inv(ori, [0, 0, -1])
        # Velocity Command
        obs += self.command_generator()

        # Joint Pose and Velocity
        states = self.client.getJointStates(
            self.robot,
            [j for j in range(self.n)]
        )
        q, dq = [], []
        for i, state in enumerate(states):
            q.append(state[0] - self.q_init_arr[i])
            dq.append(state[1])
        obs += list_to_list(q, self.joints, self.joints_isaac)
        obs += list_to_list(dq, self.joints, self.joints_isaac)
        # Last Action
        obs += list_to_list(self.action, self.joints, self.joints_isaac)
        return obs

    def update_command(self, vel_cmd: list) -> None:
        self.command_generator.set_vel_cmd(vel_cmd)

    def close(self) -> None:
        self.client.disconnect()
