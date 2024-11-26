config = {
    "H": 50,
    "sim_f": 200,  # [Hz]
    "control_f": 50,  # [Hz]
    "command": [0, 0, 0],  # [vy, vy, yaw_rate]
    "Kp": 120, # Might want to change this to 120
    "Kp": 60, # Might want to change this to 120
    "Kd": 1.5, 
    "Ka": 0.2,
    "tau_lim": 45,  # [Nm]
    "saturation_lim": 45,
    "vel_lim": 30,
    "lateral_friction": 0.4, #0.8, was 1
    "rolling_friction": 0.4, #0.6, was 1
    "init_duration_s": 1,
    "sim_duration_s": 12,
    "Kp_joints": {
        "fr.hx": 120,
        "fl.hx": 120,
        "hr.hx": 120,
        "hl.hx": 120,
        "fr.hy": 120,
        "fl.hy": 120,
        "hr.hy": 120,
        "hl.hy": 120,
        "fr.kn": 120,
        "fl.kn": 120,
        "hr.kn": 120,
        "hl.kn": 120,
    },
    "Kd_joints": {
        "fr.hx": 1.5,
        "fl.hx": 1.5,
        "hr.hx": 1.5,
        "hl.hx": 1.5,
        "fr.hy": 1.5,
        "fl.hy": 1.5,
        "hr.hy": 1.5,
        "hl.hy": 1.5,
        "fr.kn": 1.5,
        "fl.kn": 1.5,
        "hr.kn": 1.5,
        "hl.kn": 1.5,
    },
    "q_stance": {
        "fr.hx": -0.1,
        "fl.hx": 0.1,
        "hr.hx": -0.1,
        "hl.hx": 0.1,
        "fr.hy": 0.9,
        "fl.hy": 0.9,
        "hr.hy": 1.1,
        "hl.hy": 1.1,
        "fr.kn": -1.5,
        "fl.kn": -1.5,
        "hr.kn": -1.5,
        "hl.kn": -1.5,
    },
    "isaac_joint_order": {
        "fr.hx": 1,
        "fl.hx": 0,
        "hr.hx": 3,
        "hl.hx": 2,
        "fr.hy": 5,
        "fl.hy": 4,
        "hr.hy": 7,
        "hl.hy": 6,
        "fr.kn": 9,
        "fl.kn": 8,
        "hr.kn": 11,
        "hl.kn": 10,
    }
}
