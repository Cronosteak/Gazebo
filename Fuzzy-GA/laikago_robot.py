import pybullet as p
import pybullet_data
import os
import time
import numpy as np  

class LaikagoRobot:
    def get_camera_image(self, width=320, height=240, fov=60, near=0.05, far=5.0):
        """
        Devuelve la imagen RGB que "vería" una cámara montada en la base del robot mirando hacia adelante.
        """
        if not self._connected:
            self.connect()
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        # Dirección hacia adelante en el marco del robot (eje x)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        up_vec = [0, 0, 1]  # Siempre hacia arriba global
        camera_eye = [pos[0], pos[1], pos[2] + 0.15]  # un poco sobre la base
        camera_target = [camera_eye[0] + forward_vec[0],
                        camera_eye[1] + forward_vec[1],
                        camera_eye[2] + forward_vec[2]]
        view_matrix = p.computeViewMatrix(camera_eye, camera_target, up_vec)
        aspect = width / height
        proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        img = p.getCameraImage(width, height, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = img[2]  # RGB
        return rgb_array
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
        # Agregar bandera (cubo rojo) en la posición objetivo (2.0, 0, 0.25)
        flag_id = p.loadURDF("cube_small.urdf", [2.0, 0, 0.25], globalScaling=0.2, useFixedBase=True)
        # Cambiar color visual a rojo
        p.changeVisualShape(flag_id, -1, rgbaColor=[1, 0, 0, 1])
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_names = [p.getJointInfo(self.robot, i)[1].decode('utf-8') for i in range(self.num_joints)]
        for i, name in enumerate(self.joint_names):
            print(f"  {i}: {name}")
        self.motor_indices = [i for i in range(self.num_joints) if p.getJointInfo(self.robot, i)[2] == p.JOINT_REVOLUTE]
        # Mapeo seguro de índices por nombre (después de cargar el robot)
        name_to_index = {}
        for i in range(self.num_joints):
              ji = p.getJointInfo(self.robot, i)
              name_to_index[ji[1].decode()] = i
        # Ajusta los nombres a los del URDF real de Laikago
        self.order_FR = [name_to_index[n] for n in [
            "FR_hip_motor_2_chassis_joint", "FR_upper_leg_2_hip_motor_joint", "FR_lower_leg_2_upper_leg_joint"]]
        self.order_FL = [name_to_index[n] for n in [
            "FL_hip_motor_2_chassis_joint", "FL_upper_leg_2_hip_motor_joint", "FL_lower_leg_2_upper_leg_joint"]]
        self.order_RR = [name_to_index[n] for n in [
            "RR_hip_motor_2_chassis_joint", "RR_upper_leg_2_hip_motor_joint", "RR_lower_leg_2_upper_leg_joint"]]
        self.order_RL = [name_to_index[n] for n in [
            "RL_hip_motor_2_chassis_joint", "RL_upper_leg_2_hip_motor_joint", "RL_lower_leg_2_upper_leg_joint"]]
        self.motor_indices = self.order_FR + self.order_FL + self.order_RR + self.order_RL
        self._connected = True

    def reset(self, joint_positions=None):
        if not self._connected:
            self.connect()
        # Pose articular inicial estable (patas dobladas hacia abajo)
        # Orden: FR, FL, RR, RL (cada una: cadera, muslo, rodilla)
        if joint_positions is None:
            joint_positions = [0.0, 0.8, -1.5,   # FR
                              0.0, 0.8, -1.5,   # FL
                              0.0, 0.8, -1.5,   # RR
                              0.0, 0.8, -1.5]   # RL
        if len(joint_positions) != len(self.motor_indices):
            raise ValueError(f"joint_positions debe tener {len(self.motor_indices)} elementos (uno por motor)")
        for idx, pos in zip(self.motor_indices, joint_positions):
            p.resetJointState(self.robot, idx, pos)

    def step(self):
        if not self._connected:
            self.connect()
        p.stepSimulation()
        if self.gui:
            time.sleep(1.0 / 240.0)

    def set_motor_positions(self, positions):
       
        if not self._connected:
            self.connect()
        # Ganancias PD más suaves para evitar sobreimpulso
        KP = 0.5
        KV = 0.6
        # Fuerzas diferenciadas por tipo de articulación
        # Orden por pata: [cadera(abducción), muslo(pitch), rodilla(knee)]
        FORCE_ABD = 6.0   # menor para que no desestabilice lateralmente
        FORCE_HIP = 12.0  # media para avanzar
        FORCE_KNEE = 18.0 # mayor para sostener el peso
        MAX_VEL = 2.0     # limitar velocidad para evitar “disparos”

        for i, (idx, pos) in enumerate(zip(self.motor_indices, positions)):
            info = p.getJointInfo(self.robot, idx)
            low, high = info[8], info[9]  # límites articulares
            pos = float(np.clip(pos, low, high))
            # Selección de fuerza por tipo (0=cadera,1=muslo,2=rodilla)
            tipo = i % 3
            if tipo == 0:
                fuerza = FORCE_ABD
            elif tipo == 1:
                fuerza = FORCE_HIP
            else:
                fuerza = FORCE_KNEE
            p.setJointMotorControl2(
                self.robot, idx, p.POSITION_CONTROL,
                targetPosition=pos,
                force=fuerza,
                positionGain=KP,
                velocityGain=KV,
                maxVelocity=MAX_VEL
            )

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
