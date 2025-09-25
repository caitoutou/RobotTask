import os

from XRobot.robots.base import *

ASSET_DIR = os.path.join(os.path.dirname(__file__), '../assets')


class UR5e(BaseRobot):
    """ UR5e XRobot base class. """

    def __init__(self,
                 scene='default',
                 manipulator='UR5e',
                 gripper=None,
                 mount=None
                 ):
        super().__init__(
            scene=scene,
            mount=mount,
            manipulator=manipulator,
            gripper=gripper,
            attached_body='0_attachment',
        )
        self.arm_joint_names = {self.agents[0]: ['0_shoulder_pan_joint', '0_shoulder_lift_joint', '0_elbow_joint', '0_wrist_1_joint', '0_wrist_2_joint', '0_wrist_3_joint']}
        self.arm_actuator_names = {self.agents[0]: ['0_actuator1', '0_actuator2', '0_actuator3', '0_actuator4', '0_actuator5', '0_actuator6']}
        self.base_link_name = {self.agents[0]: '0_base'}
        self.end_name = {self.agents[0]: '0_attachment'}

        self.pos_max_bound = np.array([0.6, 0.2, 0.37])
        self.pos_min_bound = np.array([0.3, -0.2, 0.12])

    @property
    def init_qpos(self):
        """ Robot's init joint position. """
        return {self.agents[0]: np.array([0.0, -np.pi / 2.0, np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0])}


# class UR5eConveyor(UR5e):
#     def __init__(self):
#         super().__init__(scene='default',
#                          gripper='RobotiqGripper',
#                          mount='cylinder')
#
#     def add_assets(self):
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/conveyor belt/conveyor belt.xml')
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/carton/carton.xml')
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')
#         self.mjcf_generator.set_node_attrib('body', 'carton', {'pos': '1.1 -0.3 0.49'})
#         self.mjcf_generator.set_node_attrib('body', 'green_block', {'pos': '1.6 0.43 0.45'})
#
#     @property
#     def init_qpos(self):
#         """ Robot's init joint position. """
#         return {self.agents[0]: np.array([1.46345588e-05, -6.87047296e-01, 2.10020717e+00, -2.98390247e+00,
#                                           -1.57080312e+00, 1.57079752e+00])}


class UR5eGrasp(UR5e):
    def __init__(self):
        super().__init__(scene='grasping',
                         gripper='RobotiqGripper',
                         mount=None)

        self.end_name = {self.agents[0]: '0_eef'}

    def add_assets(self):
        # self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/red_cube.xml')
        # self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')
        # self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/blue_cube.xml')
        self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/box.xml')
        # self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/box_robot.xml')
        # self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/realsense_d435/realsense.xml', parent_body_name='worldbody')

    @property
    def init_qpos(self):
        """ Robot's init joint position. """
        # return {self.agents[0]: np.array([0.57131313, -1.58681262,  1.45363338, -1.43761664, -1.57079275,  1.29926298])}
        return {self.agents[0]: np.array([0.67131313, -1.58681262,  1.45363338, -1.43761664, -1.57079275,  0])}


if __name__ == '__main__':
    robot = UR5eGrasp()
    print(robot.get_arm_qpos())
    print(robot.get_mass_matrix())
    print(robot.get_coriolis_gravity_compensation())
    print(robot.get_end_xpos())
    print(robot.get_end_xmat())
    print(robot.get_end_xquat())
    print(robot.get_full_jac())

