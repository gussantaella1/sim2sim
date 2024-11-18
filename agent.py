import numpy as np
import onnxruntime as ort


class Agent:
    def __init__(self, policy: str, H: int = 50) -> None:
        print("G is being overwritten check: ", H)
        # Load onnx policy
        self.ort_session = ort.InferenceSession(policy)
        # Set up history tracking
        self.lin_vel = np.zeros((3, H), dtype=np.float32)  # Linear velocity
        self.ang_vel = np.zeros((3, H), dtype=np.float32)  # Angular velocity
        self.proj_g = np.zeros((3, H), dtype=np.float32)
        self.vel_cmd = np.zeros((3, H), dtype=np.float32)
        self.q = np.zeros((12, H), dtype=np.float32)
        self.dq = np.zeros((12, H), dtype=np.float32)
        self.last_action = np.zeros((12, H), dtype=np.float32)
        self.obs_list = [
            self.lin_vel,  # Include linear velocity
            self.ang_vel,  # Include angular velocity
            self.proj_g,
            self.vel_cmd,
            self.q,
            self.dq,
            self.last_action
        ]
        self.sizes = [obs.shape[0] for obs in self.obs_list]
        print("Sizes as defined here: ", self.sizes)
        self.H = H
        print("H as defined here: ", self.H)

    def __call__(self, obs: list) -> np.ndarray:
        obs_hist = self.get_flat_obs(obs)
        print("Shape of obs_hist:", obs_hist.shape)
        #OG:
        #outputs = self.ort_session.run(None, {"obs": obs_hist})
        outputs = self.ort_session.run(None, {"state_distory": obs_hist})
        print("Model outputs:", outputs)

        action = outputs[0].flatten()

        #estimate = outputs[1].flatten()
        #return action.tolist(), estimate.tolist()
        return action.tolist()




    def get_flat_obs(self, obs) -> np.ndarray:
        """
        Converts the input observation into a flat array with full history.
        """
        idx = 0
        print("self.sizes prior to sum: ", self.sizes)
        print("self.H in obs.flat:", self.H )
        print("sum(self.sizes):", sum(self.sizes) )

        obs_flat = np.zeros((1, self.H * sum(self.sizes)), dtype=np.float32)
        print("Initialized obs_flat shape:", obs_flat.shape)

        for i in range(len(self.obs_list)):
            ob, size = self.obs_list[i], self.sizes[i]
            # Shift history buffer to make room for new observation
            ob = np.roll(ob, 1, axis=1)
            ob[:, 0] = obs[idx:idx + size]  # Update with latest observation
            # Flatten buffer and insert into `obs_flat`
            ob_flat = ob.copy().transpose().reshape((1, -1))
            obs_flat[:, idx * self.H:(idx + size) * self.H] = ob_flat
            self.obs_list[i] = ob  # Save updated buffer
            idx += size

        print("obs_flat shape:", obs_flat.shape)  # Debugging shape
        return np.float32(obs_flat)


'''
    def get_flat_obs(self, obs) -> np.ndarray:
        idx = 0
        obs_flat = np.zeros((1, self.H * sum(self.sizes)))

        #obs_flat = np.zeros((1, self.H * sum(self.sizes)), dtype=np.float32)

        for i in range(len(self.obs_list)):
            ob, size = self.obs_list[i], self.sizes[i]
            ob = np.roll(ob, 1, axis=1)
            ob[:, 0] = obs[idx:idx + size]
            ob_flat = ob.copy().transpose().reshape((1, -1))
            obs_flat[:, idx * self.H:(idx + size) * self.H] = ob_flat
            self.obs_list[i] = ob
            idx += size
        # print(obs_flat[:, 60:180])
        return np.float32(obs_flat)
        # np.float32(np.random.randn(*obs_flat.shape))

'''


