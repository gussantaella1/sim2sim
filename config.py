config = {
    "sim_f": 200,  # [Hz]
    "control_f": 50,  # [Hz]
    "command": [0, 0, 0],  # [vy, vy, yaw_rate]
    "Kp": 0.01,
    "Kd": 0.0001,
    "Ka": 0.25,
    "tau_lim": 50,  # [Nm]
    "lateral_friction": 0.8,
    "rolling_friction": 0.6,
    "init_duration_s": 5,
    "q_stance": {
        "FR_hip_joint": -0.1,
        "FL_hip_joint": 0.1,
        "RR_hip_joint": -0.1,
        "RL_hip_joint": 0.1,
        "FR_thigh_joint": 0.8,
        "FL_thigh_joint": 0.8,
        "RR_thigh_joint": 1.0,
        "RL_thigh_joint": 1.0,
        "FR_calf_joint": -1.5,
        "FL_calf_joint": -1.5,
        "RR_calf_joint": -1.5,
        "RL_calf_joint": -1.5,
    },
    "isaac_joint_order": {
        "FR_hip_joint": 1,
        "FL_hip_joint": 0,
        "RR_hip_joint": 3,
        "RL_hip_joint": 2,
        "FR_thigh_joint": 5,
        "FL_thigh_joint": 4,
        "RR_thigh_joint": 7,
        "RL_thigh_joint": 6,
        "FR_calf_joint": 9,
        "FL_calf_joint": 8,
        "RR_calf_joint": 11,
        "RL_calf_joint": 10,
    }
}
