from agent import Agent
from config import config
from env import Env
from plot import Logger
from utils import list_to_dict
#import onnx


#agent = Agent("models/policy_V1.onnx", H=config["H"])

#model = onnx.load("models/policy_V1.onnx")
#print(onnx.helper.printable_graph(model.graph))

agent = Agent("models/policy_V2.onnx", H = config["H"])

#agent = Agent("models/policy_V1.onnx", H = 50)
env = Env(cfg=config)


log = Logger(Ka=config["Ka"], f=config["control_f"])


#Observation order:
# 1) Linear velocity (3 terms) 0:3
# 2) Angular velocity (3 terms) 3:6
# 3) Projected gravity (3 terms) 6:9
# 4) Velocity command (3 terms) 9:12
# 5) Joint positions (12 terms) 12:24
# 6) Joint velocities (12 terms) 24:36
# 7) Actions (12 terms) 36:48


action = [0 for _ in range(env.n)]
for _ in range(config["init_duration_s"] * env.control_f):

    obs, torque, q_des = env.step_init()
    '''
    log.log(
        proj_g=obs[6:9],
        vel_cmd=obs[9:12],
        q=list_to_dict(obs[12:24], env.joints_isaac),
        dq=list_to_dict(obs[24:36], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
    '''
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
    #print("Length of obs:", len(obs))

    #action, estimate = agent(obs)
    action = agent(obs)

    #Use when debugging:
    #action = [0]*12

    #print("estimate: ", estimate)
    '''
    log.log(
        proj_g=obs[6:9],
        vel_cmd=obs[9:12],
        q=list_to_dict(obs[12:24], env.joints_isaac),
        dq=list_to_dict(obs[24:36], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
    '''

    '''
    print("\nInput to logger:")
    print(f"  proj_g (Projected Gravity):  {obs[6:9]}")
    print(f"  vel_cmd (Velocity Command):  {obs[9:12]}")
    print(f"  q (Joint Positions):\n    {list_to_dict(obs[12:24], env.joints_isaac)}")
    print(f"  dq (Joint Velocities):\n    {list_to_dict(obs[24:36], env.joints_isaac)}")
    print(f"  action (Actions):\n    {list_to_dict(action, env.joints_isaac)}")
    print(f"  q_offset (Joint Offsets):\n    {env.q_init}")
    print(f"  torque (Torques):\n    {list_to_dict(torque, env.joints_isaac)}")
    print(f"  q_des (Desired Positions):\n    {list_to_dict(q_des, env.joints_isaac)}")
    '''

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
    action = agent(obs)

    #Use when debugging:
    #action = [0]*12

    #print("estimate: ", estimate)


    # Fix order of observations.

    log.log(
        proj_g=obs[6:9],
        vel_cmd=obs[9:12],
        q=list_to_dict(obs[12:24], env.joints_isaac),
        dq=list_to_dict(obs[24:36], env.joints_isaac),
        action=list_to_dict(action, env.joints_isaac),
        q_offset=env.q_init,
        torque=list_to_dict(torque, env.joints_isaac),
        q_des=list_to_dict(q_des, env.joints_isaac),
    )
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


try:
    print("Attempting to plot...")
    log.plot()
except Exception as e:
    print(f"Error during plot: {e}")

env.close()
