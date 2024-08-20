import numpy as np
import onnxruntime as ort


class Agent:
    def __init__(self, policy: str, H: int = 10) -> None:
        # Load onnx policy
        self.ort_session = ort.InferenceSession(policy)
        # Set up history tracking
        self.proj_g = np.zeros((3, H), dtype=np.float32)
        self.vel_cmd = np.zeros((3, H), dtype=np.float32)
        self.q = np.zeros((12, H), dtype=np.float32)
        self.dq = np.zeros((12, H), dtype=np.float32)
        self.last_action = np.zeros((12, H), dtype=np.float32)
        self.obs_list = [
            self.proj_g,
            self.vel_cmd,
            self.q,
            self.dq,
            self.last_action
        ]
        self.sizes = [obs.shape[0] for obs in self.obs_list]
        self.H = H

    def __call__(self, obs: list) -> np.ndarray:
        obs_hist = self.get_flat_obs(obs)
        action = self.ort_session.run(None, {"obs": obs_hist})[0].flatten()
        return action.tolist()

    def get_flat_obs(self, obs) -> np.ndarray:
        idx = 0
        obs_flat = np.zeros((1, self.H * sum(self.sizes)))
        for i in range(len(self.obs_list)):
            ob, size = self.obs_list[i], self.sizes[i]
            ob = np.roll(ob, 1, axis=1)
            ob[:, 0] = obs[idx:idx + size]
            ob_flat = ob.copy().reshape((1, -1))
            obs_flat[:, idx * self.H:(idx + size) * self.H] = ob_flat
            self.obs_list[i] = ob
            idx += size
        return np.float32(obs_flat)
