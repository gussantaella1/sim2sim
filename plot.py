import matplotlib.pyplot as plt
import os

#import matplotlib

#print("Backend for this wacky: ", matplotlib.get_backend())


#OG naming convention for joints:        
#"FR_hip_joint": [],
#"FL_hip_joint": [],
#"RR_hip_joint": [],
#"RL_hip_joint": [],
#"FR_thigh_joint": [],
#"FL_thigh_joint": [],
#"RR_thigh_joint": [],
#"RL_thigh_joint": [],
#"FR_calf_joint": [],
#"FL_calf_joint": [],
#"RR_calf_joint": [],
#"RL_calf_joint": [],
            

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
            "fr.hx": [],
            "fl.hx": [],
            "hr.hx": [],
            "hl.hx": [],
            "fr.hy": [],
            "fl.hy": [],
            "hr.hy": [],
            "hl.hy": [],
            "fr.kn": [],
            "fl.kn": [],
            "hr.kn": [],
            "hl.kn": [],
        }
        self.q_des = {
            "fr.hx": [],
            "fl.hx": [],
            "hr.hx": [],
            "hl.hx": [],
            "fr.hy": [],
            "fl.hy": [],
            "hr.hy": [],
            "hl.hy": [],
            "fr.kn": [],
            "fl.kn": [],
            "hr.kn": [],
            "hl.kn": [],
        }
        # Joint Velocity
        self.dq = {
            "fr.hx": [],
            "fl.hx": [],
            "hr.hx": [],
            "hl.hx": [],
            "fr.hy": [],
            "fl.hy": [],
            "hr.hy": [],
            "hl.hy": [],
            "fr.kn": [],
            "fl.kn": [],
            "hr.kn": [],
            "hl.kn": [],
        }
        # Action
        self.action = {
            "fr.hx": [],
            "fl.hx": [],
            "hr.hx": [],
            "hl.hx": [],
            "fr.hy": [],
            "fl.hy": [],
            "hr.hy": [],
            "hl.hy": [],
            "fr.kn": [],
            "fl.kn": [],
            "hr.kn": [],
            "hl.kn": [],
        }
        self.torque = {
            "fr.hx": [],
            "fl.hx": [],
            "hr.hx": [],
            "hl.hx": [],
            "fr.hy": [],
            "fl.hy": [],
            "hr.hy": [],
            "hl.hy": [],
            "fr.kn": [],
            "fl.kn": [],
            "hr.kn": [],
            "hl.kn": [],
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
        #print(f"Joint Position Axes: {self.ax_q}")

        # Joint Velocity
        self.fig_dq, self.ax_dq = plt.subplots(3, 1, **subplot_kw_args)
        #print(f"Joint Velocity Axes: {self.ax_dq}")

        # Action
        self.fig_action, self.ax_action = plt.subplots(3, 1, **subplot_kw_args)
        #print(f"Action Axes: {self.ax_action}")

        # Torque
        self.fig_torque, self.ax_torque = plt.subplots(3, 1, **subplot_kw_args)
        #print(f"Torque Axes: {self.ax_torque}")

        

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
        #print("STARTING MY COOL PLOTS")

        # Define a directory to save the plots
        output_dir = "plots"
        os.makedirs(output_dir, exist_ok=True)

        key_mapping = {
            "fr.hx": "FR_hip_joint",
            "fl.hx": "FL_hip_joint",
            "hr.hx": "RR_hip_joint",
            "hl.hx": "RL_hip_joint",
            "fr.hy": "FR_thigh_joint",
            "fl.hy": "FL_thigh_joint",
            "hr.hy": "RR_thigh_joint",
            "hl.hy": "RL_thigh_joint",
            "fr.kn": "FR_calf_joint",
            "fl.kn": "FL_calf_joint",
            "hr.kn": "RR_calf_joint",
            "hl.kn": "RL_calf_joint",
        }    

        # Overwrite self.q and other dictionaries with old naming convention
        self.q = {key_mapping[k]: v for k, v in self.q.items()}
        self.q_des = {key_mapping[k]: v for k, v in self.q_des.items()}
        self.dq = {key_mapping[k]: v for k, v in self.dq.items()}
        self.action = {key_mapping[k]: v for k, v in self.action.items()}
        self.torque = {key_mapping[k]: v for k, v in self.torque.items()}


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
        self.fig_proj_g.savefig(os.path.join(output_dir, "projected_gravity.png"))

        # Velocity Command
        for k, v in self.vel_cmd.items():
            self.ax_vel_cmd.plot(self.t, v, label=k)
        self.ax_vel_cmd.legend(loc="upper right")
        self.ax_vel_cmd.set_title("Velocity Command")
        self.ax_vel_cmd.set_ylabel("Velocity [m/s, rad/s]")
        self.ax_vel_cmd.set_xlabel("Time [s]")
        self.fig_vel_cmd.savefig(os.path.join(output_dir, "velocity_command.png"))

        # Joint Position
        for k, v in self.q.items():

            #print("Here for the thousand time!")
            c = styles[k.split("_")[0]]

            # Check types and lengths
            #print(f"Key: {k}")
            #print(f"Time vector type: {type(self.t)}, element types: {[type(t) for t in self.t[:5]]}")
            #print(f"Data vector type: {type(v)}, element types: {[type(val) for val in v[:5]]}")
            

            #try:
            #    c = styles[k.split("_")[0]]
            #    print(f"Key: {k}, Style: {c}")
            #except KeyError as e:
            #    print(f"KeyError for joint {k}: {e}")
            #    continue  # Skip this joint if there's an error

            # Print the lengths of time vector and joint positions for better intuition
            #print(f"Length of Time Vector (self.t): {len(self.t)}")
            #print(f"Length of Joint Positions for {k} (self.q[{k}]): {len(v)}")
            #print(f"Length of Desired Positions for {k} (self.q_des[{k}]): {len(self.q_des[k])}")

            #label = k.split("_joint")[0]

            # Debug statements
            #print(f"Time Vector (self.t): Length={len(self.t)}, Values={self.t[:5]}...")  # Print first 5 values
            #print(f"Joint Data (v): Key={k}, Length={len(v)}, Values={v[:5]}...")  # Print first 5 values
            #print(f"Style (c): {c}, Label: {label}")n


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
        self.fig_q.savefig(os.path.join(output_dir, "joint_positions.png"))

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
        self.fig_dq.savefig(os.path.join(output_dir, "joint_velocity.png"))

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
        self.fig_action.savefig(os.path.join(output_dir, "actions.png"))

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

        self.fig_torque.savefig(os.path.join(output_dir, "torque.png"))

        print(f"Plots saved in the directory: {output_dir}")
        #plt.show()
