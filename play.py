from agent import Agent
from env import Env

agent = Agent("models/model_94000.onnx", H=10)
env = Env()

action = [0 for _ in range(12)]
for _ in range(1000):
    obs = env.step(action)
    action = agent(obs)
