"""
Pot push task.
- Action space: 3 DOF with gripper action flipped.
"""


import os.path
import numpy as np

BASE_DIR = "/".join(str.split(__file__, "/")[:-1])
current_dir = os.path.dirname(os.path.realpath(__file__))

from multicam_server.topic_utils import IMTopic

from widowx_envs.widowx_env import VR_WidowX
from widowx_envs.control_loops import TimedLoop
from widowx_envs.policies.vr_teleop_policy import VRTeleopPolicy


class BoxBound(object):
    def __init__(self, min_pos, max_pos):
        self.min_pos = min_pos
        self.max_pos = max_pos

    def project(self, pos):
        return np.clip(pos, self.min_pos, self.max_pos)


ee_pos_bound = BoxBound(np.array([0.25, -0.15, 0.17]), np.array([0.31, 0.11, 0.29]))

env_params = {
    "camera_topics": [
        IMTopic.from_dict(
            {
                "name": "/blue/image_raw",
                "flip": False,
            }
        )
    ],
    "gripper_attached": "custom",
    "gripper_flip_value": False,
    "skip_move_to_neutral": True,
    "move_to_rand_start_freq": -1,
    "action_mode": "3trans1rot",
    "fix_zangle": 0.1,
    "move_duration": 0.2,
    "adaptive_wait": True,
    "override_workspace_boundaries": [
        ee_pos_bound.min_pos.tolist() + [-0.57, 0],
        ee_pos_bound.max_pos.tolist() + [0.57, 0],
    ],
    "action_clipping": "xyzrot",
    "neutral_joint_angles": [0.33, 0.17, -0.34, 0.0, 1.8, 0.34],
}

agent = {
    "type": TimedLoop,
    "env": (VR_WidowX, env_params),
    "recreate_env": (False, 1),
    "T": 300,
    "image_height": 480,
    "image_width": 640,
    "make_final_gif": False,
    "video_format": "mp4",
}

policy = {
    "type": VRTeleopPolicy,
}

config = {
    "current_dir": current_dir,
    "collection_metadata": current_dir + "/collection_metadata.json",
    "start_index": 0,
    "end_index": 500,
    "agent": agent,
    "policy": policy,
    "save_format": ["raw"],
    "make_diagnostics": True,
    "record_floor_height": False,
}
