import matplotlib.pyplot as plt


class Logger:
    def __init__(self, Ka: float = 0.25, f: float = 50) -> None:
        # Data
        self.Ka = Ka
        # Time
        self.dt = 1. / f
        self.t = []
        self.counter = 0
        # Projected Gravity
        self.proj_g = {
            "gx": [],
            "gy": [],
            "gz": [],
        }
        # Velocity Command
        self.vel_cmd = {
            "vx": [],
            "vy": [],
            "yaw rate": [],
        }
        # Joint Position
        self.q = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        self.q_des = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        # Joint Velocity
        self.dq = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        # Action
        self.action = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }
        self.torque = {
            "FR_hip_joint": [],
            "FL_hip_joint": [],
            "RR_hip_joint": [],
            "RL_hip_joint": [],
            "FR_thigh_joint": [],
            "FL_thigh_joint": [],
            "RR_thigh_joint": [],
            "RL_thigh_joint": [],
            "FR_calf_joint": [],
            "FL_calf_joint": [],
            "RR_calf_joint": [],
            "RL_calf_joint": [],
        }

        # Plotters
        subplot_kw_args = {
            "sharex": True,
            # "sharey": True,
        }
        # Projected Gravity
        self.fig_proj_g, self.ax_proj_g = plt.subplots()
        # Velocity Command
        self.fig_vel_cmd, self.ax_vel_cmd = plt.subplots()
        # Joint Position
        self.fig_q, self.ax_q = plt.subplots(3, 1, **subplot_kw_args)
        # Joint Velocity
        self.fig_dq, self.ax_dq = plt.subplots(3, 1, **subplot_kw_args)
        # Action
        self.fig_action, self.ax_action = plt.subplots(3, 1, **subplot_kw_args)
        # Torque
        self.fig_torque, self.ax_torque = plt.subplots(3, 1, **subplot_kw_args)

    def log(
            self,
            proj_g: list,
            vel_cmd: list,
            q: dict,
            dq: dict,
            action: dict,
            q_offset: dict,
            torque: dict,
            q_des: dict,
    ) -> None:
        # Time
        self.t.append(self.counter * self.dt)
        self.counter += 1
        # Projected Gravity
        self.proj_g["gx"].append(proj_g[0])
        self.proj_g["gy"].append(proj_g[1])
        self.proj_g["gz"].append(proj_g[2])
        # Velocity Command
        self.vel_cmd["vx"].append(vel_cmd[0])
        self.vel_cmd["vy"].append(vel_cmd[1])
        self.vel_cmd["yaw rate"].append(vel_cmd[2])
        # Joint Position
        for k, v in q.items():
            self.q[k].append(v + q_offset[k])
        for k, v in q_des.items():
            self.q_des[k].append(v)
        # Joint Velocity
        for k, v in dq.items():
            self.dq[k].append(v)
        # Action
        for k, v in action.items():
            self.action[k].append(self.Ka * v)
        # Torque
        for k, v in torque.items():
            self.torque[k].append(v)

        def plot(self) -> None:
        styles = {
            "FR": "b",  # "tab:blue",
            "FL": "g",  # "tab:orange",
            "RR": "r",  # "tab:green",
            "RL": "m",  # "tab:red",
        }
        # Projected Gravity
        for k, v in self.proj_g.items():
            self.ax_proj_g.plot(self.t, v, label=k)
        self.ax_proj_g.legend(loc="upper right")
        self.ax_proj_g.set_title("Projected Gravity")
        self.ax_proj_g.set_ylabel("Force [N]")
        self.ax_proj_g.set_xlabel("Time [s]")
        # Velocity Command
        for k, v in self.vel_cmd.items():
            self.ax_vel_cmd.plot(self.t, v, label=k)
        self.ax_vel_cmd.legend(loc="upper right")
        self.ax_vel_cmd.set_title("Velocity Command")
        self.ax_vel_cmd.set_ylabel("Velocity [m/s, rad/s]")
        self.ax_vel_cmd.set_xlabel("Time [s]")
        # Joint Position
        for k, v in self.q.items():
            c = styles[k.split("_")[0]]
            if "hip" in k:
                self.ax_q[0].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_q[0].legend(loc="upper right")
            elif "thigh" in k:
                self.ax_q[1].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_q[1].legend(loc="upper right")
            elif "calf" in k:
                self.ax_q[2].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_q[2].legend(loc="upper right")
        # Desired
        for k, v in self.q_des.items():
            c = styles[k.split("_")[0]] + "--"
            if "hip" in k:
                self.ax_q[0].plot(self.t, v, c, label=k.split("_joint")[0]+"_des")
                self.ax_q[0].legend(loc="upper right")
            elif "thigh" in k:
                self.ax_q[1].plot(self.t, v, c, label=k.split("_joint")[0]+"_des")
                self.ax_q[1].legend(loc="upper right")
            elif "calf" in k:
                self.ax_q[2].plot(self.t, v, c, label=k.split("_joint")[0]+"_des")
                self.ax_q[2].legend(loc="upper right")
        self.ax_q[0].set_title("Joint Position")
        self.ax_q[1].set_ylabel("Angle [rad]")
        self.ax_q[2].set_xlabel("Time [s]")
        # Joint Velocity
        for k, v in self.dq.items():
            c = styles[k.split("_")[0]]
            if "hip" in k:
                self.ax_dq[0].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_dq[0].legend(loc="upper right")
            elif "thigh" in k:
                self.ax_dq[1].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_dq[1].legend(loc="upper right")
            elif "calf" in k:
                self.ax_dq[2].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_dq[2].legend(loc="upper right")
        self.ax_dq[0].set_title("Joint Velocity")
        self.ax_dq[1].set_ylabel("Angular Rate [rad/s]")
        self.ax_dq[2].set_xlabel("Time [s]")
        # Action
        for k, v in self.action.items():
            c = styles[k.split("_")[0]]
            if "hip" in k:
                self.ax_action[0].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_action[0].legend(loc="upper right")
            elif "thigh" in k:
                self.ax_action[1].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_action[1].legend(loc="upper right")
            elif "calf" in k:
                self.ax_action[2].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_action[2].legend(loc="upper right")
        self.ax_action[0].set_title("Action")
        self.ax_action[1].set_ylabel("Angle Offset [rad]")
        self.ax_action[2].set_xlabel("Time [s]")
        # Torque
        for k, v in self.torque.items():
            c = styles[k.split("_")[0]]
            if "hip" in k:
                self.ax_torque[0].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_torque[0].legend(loc="upper right")
            elif "thigh" in k:
                self.ax_torque[1].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_torque[1].legend(loc="upper right")
            elif "calf" in k:
                self.ax_torque[2].plot(self.t, v, c, label=k.split("_joint")[0])
                self.ax_torque[2].legend(loc="upper right")
        self.ax_torque[0].set_title("Applied Torque")
        self.ax_torque[1].set_ylabel("Torque [Nm]")
        self.ax_torque[2].set_xlabel("Time [s]")
        plt.show()
