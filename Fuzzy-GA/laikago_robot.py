import pybullet as p
import pybullet_data
import os
import time


class LaikagoRobot:
    def __init__(self, gui=True):
        self.client = None
        self.gui = gui
        self.robot = None
        self.num_joints = 0
        self.joint_names = []
        self.motor_indices = []
        self._connected = False

    def connect(self):
        if self._connected:
            return
        if self.gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        laikago_urdf = os.path.join(pybullet_data.getDataPath(), "laikago/laikago_toes.urdf")
        # Rotar 90° en X y 90° en Z para que quede sobre sus patas
        orn = p.getQuaternionFromEuler([1.5708, 0, 1.5708])
        self.robot = p.loadURDF(laikago_urdf, [0, 0, 0.35], orn)
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_names = [p.getJointInfo(self.robot, i)[1].decode('utf-8') for i in range(self.num_joints)]
        self.motor_indices = [i for i in range(self.num_joints) if p.getJointInfo(self.robot, i)[2] == p.JOINT_REVOLUTE]
        self._connected = True

    def reset(self, joint_positions=None):
        if not self._connected:
            self.connect()
        # Pose articular inicial estable (patas dobladas hacia abajo)
        # Valores típicos para Laikago: cadera=0, muslo=0.8, rodilla=-1.5
        if joint_positions is None:
            # Asume 12 motores: [cadera, muslo, rodilla] * 4
            joint_positions = [0.0, 0.8, -1.5] * 4
        for idx, pos in zip(self.motor_indices, joint_positions):
            p.resetJointState(self.robot, idx, pos)

    def step(self):
        if not self._connected:
            self.connect()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    def set_motor_positions(self, positions):
        if not self._connected:
            self.connect()
        for idx, pos in zip(self.motor_indices, positions):
            p.setJointMotorControl2(self.robot, idx, p.POSITION_CONTROL, targetPosition=pos)

    def get_base_position(self):
        if not self._connected:
            self.connect()
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        return pos, orn

    def get_joint_states(self):
        if not self._connected:
            self.connect()
        return [p.getJointState(self.robot, idx)[0] for idx in self.motor_indices]

    def disconnect(self):
        if self._connected:
            p.disconnect()
            self._connected = False

if __name__ == "__main__":
    robot = LaikagoRobot(gui=True)
    robot.connect()
    robot.reset()
    print("Motores:", robot.joint_names)
    try:
        while True:
            robot.step()
    except KeyboardInterrupt:
        robot.disconnect()
        print("Simulación finalizada.")
