from agent import Agent
from config import config
from env import Env
from plot import Logger
from utils import list_to_dict

agent = Agent("models/model_94000.onnx", H=config["H"])
env = Env(cfg=config)
log = Logger(Ka=config["Ka"], f=config["control_f"])

sim_time = config["sim_duration_s"]
action = [0 for _ in range(env.n)]
for _ in range(sim_time * env.control_f):
    obs = env.step(action)
    # action = agent(obs)
    log.log(
        proj_g=obs[0:3],
        vel_cmd=obs[3:6],
        q=list_to_dict(obs[6:18], env.joints_isaac),
        dq=list_to_dict(obs[18:30], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
    )
log.plot()
env.close()
