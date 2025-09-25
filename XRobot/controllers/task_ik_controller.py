import numpy as np
import mujoco
from mujoco import minimize
from .base_controller import BaseController
import XRobot.commons.transform as T


class CartesianIKController(BaseController):

    def __init__(
            self, 
            robot,
    ):
        super().__init__(robot)

        self.name = 'CARTIK'

        self.p_cart = 0.2
        self.d_cart = 0.01
        self.p_quat = 0.2
        self.d_quat = 0.01
        self.vel_des = np.zeros(3)

        self.B = np.zeros(self.dofs)
        self.K = np.zeros(self.dofs)

        self.set_jnt_params(
            b=20.0 * np.ones(self.dofs),
            k=80.0 * np.ones(self.dofs),
        )

    def step_controller(self, action):
        """
        :param: action: end pose
        :return: joint torque
        """
        ret = dict()

        if isinstance(action, np.ndarray):
            action = {self.robot.agents[0]: action}

        for agent, act in action.items():
            assert len(act) in (3, 7), "Invalid action length."

            p_goal = act[:3]
            r_goal = self.robot.init_quat[agent] if len(act) == 3 else act[3:]

            torque = self.compute_jnt_torque(
                q_des=self.ik(p_goal, r_goal, agent),
                v_des=np.zeros(self.dofs),
                q_cur=self.robot.get_arm_qpos(agent),
                v_cur=self.robot.get_arm_qvel(agent),
                agent=agent
            )
            ret[agent] = torque

        return ret

    def compute_jnt_torque(
            self,
            q_des: np.ndarray,
            v_des: np.ndarray,
            q_cur: np.ndarray,
            v_cur: np.ndarray,
            agent: str = 'arm0'
    ):
        """ robot的关节空间控制的计算公式
            Compute desired torque with XRobot dynamics modeling:
            > M(q)qdd + C(q, qd)qd + G(q) + tau_F(qd) = tau_ctrl + tau_env

        :param q_des: desired joint position
        :param v_des: desired joint velocity
        :param q_cur: current joint position
        :param v_cur: current joint velocity
        :param agent:
        :return: desired joint torque
        """

        M = self.robot.get_mass_matrix(agent)
        compensation = self.robot.get_coriolis_gravity_compensation(agent)

        acc_desire = self.K * (q_des - q_cur) + self.B * (v_des - v_cur)
        tau = np.dot(M, acc_desire) + compensation
        return tau

    def ik(self, pos, quat, agent='arm0'):
        x = self.robot.get_arm_qpos(agent)
        x_prev = x.copy()
        radius = .2
        ik_target = lambda x: self._ik_res(x, pos=pos, quat=quat, radius=radius, reg=.01, reg_target=x_prev, agent=agent)
        jac_target = lambda x, r: self._ik_jac(x, r, pos=pos, quat=quat, radius=radius, reg=.01, agent=agent)
        x, _ = minimize.least_squares(x, ik_target, self.robot.mani_joint_bounds[agent], jacobian=jac_target, eps=1e-6, verbose=0)

        return x

    def _ik_res(self, x, pos=None, quat=None, radius=6., reg=1e-3, reg_target=None, agent='arm0'):
        """Residual for inverse kinematics.

        Args:
            x: joint angles.
            pos: target position for the end effector.
            quat: target orientation for the end effector.
            radius: scaling of the 3D cross.

        Returns:
            The residual of the Inverse Kinematics task.
        """
        # Position residual.
        p_cur, r_cur = self.forward_kinematics(x, agent)
        res_pos = p_cur - pos

        # Orientation residual: quaternion difference.
        res_quat = np.empty(3)
        mujoco.mju_subQuat(res_quat, quat, r_cur)
        res_quat *= radius

        # Regularization residual.
        reg_target = self.robot.init_qpos[agent] if reg_target is None else reg_target
        res_reg = reg * (x.squeeze() - reg_target)
        
        return np.hstack((res_pos, res_quat, res_reg))

    def _ik_jac(self, x, res, pos=None, quat=None, radius=.04, reg=1e-3, agent='arm0'):
        """Analytic Jacobian of inverse kinematics residual

        Args:
            x: joint angles.
            pos: target position for the end effector.
            quat: target orientation for the end effector.
            radius: scaling of the 3D cross.

        Returns:
            The Jacobian of the Inverse Kinematics task.
        """
        # least_squares() passes the value of the residual at x which is sometimes
        # useful, but we don't need it here.
        del res

        # We can assume x has been copied into qpos
        # and that mj_kinematics has been called by ik()
        # Call mj_comPos (required for Jacobians).
        mujoco.mj_comPos(self.robot.robot_model, self.robot.kine_data)

        # Get end-effector site Jacobian.
        jac_pos = np.empty((3, self.robot.robot_model.nv))
        jac_quat = np.empty((3, self.robot.robot_model.nv))
        mujoco.mj_jacBody(self.robot.robot_model, self.robot.kine_data, jac_pos, jac_quat, self.robot.kine_data.body(self.robot.end_name[agent]).id)
        jac_pos = jac_pos[:, self.robot.arm_joint_indexes[agent]]
        jac_quat = jac_quat[:, self.robot.arm_joint_indexes[agent]]

        # Get Deffector, the 3x3 mju_subquat Jacobian
        effector_quat = T.mat_2_quat(
            self.robot.kine_data.body(self.robot.end_name[agent]).xmat.reshape(3, 3)
        )
        Deffector = np.empty((3, 3))
        mujoco.mjd_subQuat(quat, effector_quat, None, Deffector)

        # Rotate into target frame, multiply by subQuat Jacobian, scale by radius.
        target_mat = T.quat_2_mat(quat)
        mat = radius * Deffector.T @ target_mat.T
        jac_quat = mat @ jac_quat

        if self.reference == 'base':
            base2world_mat = self.robot.get_base_xmat(agent)
            jac_pos = base2world_mat.T @ jac_pos
            jac_quat = base2world_mat.T @ jac_quat

        # Regularization Jacobian.
        jac_reg = reg * np.eye(len(self.robot.arm_joint_indexes[agent]))

        return np.vstack((jac_pos, jac_quat, jac_reg))

    def set_jnt_params(self, b: np.ndarray, k: np.ndarray):
        """ Used for changing the parameters. """
        self.B = b
        self.K = k

    def compute_pd_increment(self, p_goal: np.ndarray,
                             p_cur: np.ndarray,
                             r_goal: np.ndarray,
                             r_cur: np.ndarray,
                             pd_goal: np.ndarray = np.zeros(3),
                             pd_cur: np.ndarray = np.zeros(3)):
        pos_incre = self.p_cart * (p_goal - p_cur) + self.d_cart * (pd_goal - pd_cur)
        quat_incre = self.p_quat * (r_goal - r_cur)
        return pos_incre, quat_incre

