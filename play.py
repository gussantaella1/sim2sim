from agent import Agent
from config import config
from env import Env
from plot import Logger
from utils import list_to_dict

#agent = Agent("models/model_67500.onnx", H=config["H"])
env = Env(cfg=config)


#log = Logger(Ka=config["Ka"], f=config["control_f"])

action = [0 for _ in range(env.n)]
for _ in range(config["init_duration_s"] * env.control_f):
    obs, torque, q_des = env.step_init()
    '''
    log.log(
        proj_g=obs[0:3],
        vel_cmd=obs[3:6],
        q=list_to_dict(obs[6:18], env.joints_isaac),
        dq=list_to_dict(obs[18:30], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
    '''
for _ in range(config["init_duration_s"] * env.control_f):
    obs, torque, q_des = env.step(action)
    #action, estimate = agent(obs)
    action = [0]*12
    #print("estimate: ", estimate)
    '''
    log.log(
        proj_g=obs[0:3],
        vel_cmd=obs[3:6],
        q=list_to_dict(obs[6:18], env.joints_isaac),
        dq=list_to_dict(obs[18:30], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
    '''
env.update_command([1.0, 0.0, 0.0])
for _ in range(config["sim_duration_s"] * env.control_f):
    obs, torque, q_des = env.step(action)
    #action, estimate = agent(obs)
    action = [0]*12
    #print("estimate: ", estimate)
    '''
    log.log(
        proj_g=obs[0:3],
        vel_cmd=obs[3:6],
        q=list_to_dict(obs[6:18], env.joints_isaac),
        dq=list_to_dict(obs[18:30], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
    '''
#log.plot()
env.close()
