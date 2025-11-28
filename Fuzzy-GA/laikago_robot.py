import pybullet as p
import pybullet_data
import numpy as np

class LaikagoRobot:
    # Nombres de articulaciones según impresión de PyBullet
    JOINT_NAMES = [
        'FR_hip_motor_2_chassis_joint', 'FR_upper_leg_2_hip_motor_joint', 'FR_lower_leg_2_upper_leg_joint',
        'FL_hip_motor_2_chassis_joint', 'FL_upper_leg_2_hip_motor_joint', 'FL_lower_leg_2_upper_leg_joint',
        'RR_hip_motor_2_chassis_joint', 'RR_upper_leg_2_hip_motor_joint', 'RR_lower_leg_2_upper_leg_joint',
        'RL_hip_motor_2_chassis_joint', 'RL_upper_leg_2_hip_motor_joint', 'RL_lower_leg_2_upper_leg_joint'
    ]

    # Pose inicial recomendada por el repositorio oficial (en radianes)
    INIT_POSE = {
        'hip': 0,      # abducción
        'thigh': 0.67,   # muslo adelantado
        'calf': -1.25    # rodilla hacia abajo
    }

    def __init__(self, gui=True):
        self.gui = gui
        self.client = None
        self.robot = None
        self.joint_indices = []
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
        # Girar 180 grados en Z para que mire hacia el eje X negativo
        orientation = p.getQuaternionFromEuler([np.pi/2, 0, -np.pi/2])
        self.robot = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, 0.38], baseOrientation=orientation)
        # Ajustar cámara para vista lateral (eje Y)
        if self.gui:
            # Camera follows robot from the side (lateral view)
            pos, _ = p.getBasePositionAndOrientation(self.robot)
            p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[pos[0], pos[1], 0.5])
        print("Articulaciones del robot cargado:")
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            print(f"  {i}: {info[1].decode('utf-8')}")
        # Mapear nombres a índices
        self.joint_indices = []
        for name in self.JOINT_NAMES:
            for i in range(p.getNumJoints(self.robot)):
                info = p.getJointInfo(self.robot, i)
                if info[1].decode('utf-8') == name:
                    self.joint_indices.append(i)
        self._connected = True

    def reset_pose(self):
        # Mapeo real de índices según impresión de PyBullet:
        # FR: 0,1,2 | FL: 4,5,6 | RR: 8,9,10 | RL: 12,13,14
        # Pose oficial: hip=0, thigh=0.67, calf=-1.25
        pose_map = {
            # FR
            0: 0.0,   # hip
            1: 0.67,  # thigh
            2: -1.25, # calf
            # FL
            4: 0.0,
            5: 0.67,
            6: -1.25,
            # RR
            8: 0.0,
            9: 0.67,
            10: -1.25,
            # RL
            12: 0.0,
            13: 0.67,
            14: -1.25
        }
        for idx, pos in pose_map.items():
            p.resetJointState(self.robot, idx, pos)

    def set_joint_commands(self, abduccion, muslo, rodilla, max_force=40, max_velocity=2.0):
        # abduccion, muslo, rodilla: listas de 4 valores (FL, FR, RL, RR)
        for pata in range(4):
            base = pata * 3
            p.setJointMotorControl2(self.robot, self.joint_indices[base + 0], p.POSITION_CONTROL, targetPosition=abduccion[pata], force=max_force, maxVelocity=max_velocity)
            p.setJointMotorControl2(self.robot, self.joint_indices[base + 1], p.POSITION_CONTROL, targetPosition=muslo[pata], force=max_force, maxVelocity=max_velocity)
            p.setJointMotorControl2(self.robot, self.joint_indices[base + 2], p.POSITION_CONTROL, targetPosition=rodilla[pata], force=max_force, maxVelocity=max_velocity)

    def get_base_position(self):
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        return pos, orn

    def step(self):
        p.stepSimulation()
        if self.gui:
            import time
            time.sleep(1.0 / 240.0)

    def disconnect(self):
        if self._connected:
            p.disconnect()
            self._connected = False

if __name__ == "__main__":
    robot = LaikagoRobot(gui=True)
    robot.connect()
    robot.reset_pose()
    try:
        while True:
            robot.step()
    except KeyboardInterrupt:
        robot.disconnect()
        print("Simulación finalizada.")
