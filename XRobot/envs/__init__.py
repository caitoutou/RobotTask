from .base import MujocoEnv
from .robot import RobotEnv
# from .demo_pick_place import PickAndPlaceEnv

ENVS = [
    # PickAndPlaceEnv,
]
REGISTERED_ENVS = {env.name: env for env in ENVS}