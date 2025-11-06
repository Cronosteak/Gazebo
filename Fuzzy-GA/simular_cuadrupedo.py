import pybullet as p
import pybullet_data
import time
import os

# Inicializa PyBullet en modo GUI
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    # Carga el plano
    plane_id = p.loadURDF("plane.urdf")

    # Elige el modelo a cargar: "mini_cheetah" o "laikago"
    # Usar el Laikago oficial de PyBullet
    laikago_urdf = os.path.join(pybullet_data.getDataPath(), "laikago/laikago_toes.urdf")
    orn = p.getQuaternionFromEuler([1.5708, 0, 1.5708])
    robot_id = p.loadURDF(laikago_urdf, [0, 0, 0.35], orn)

    # Obtener los índices de las articulaciones revolutas (motores)
    num_joints = p.getNumJoints(robot_id)
    motor_indices = [i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE]

    print("Simulación corriendo con Laikago. Las patas se moverán en el lugar. Presiona Ctrl+C para cerrar.")
    t = 0.0
    try:
        while True:
            # Movimiento senoide para cada motor (pies se mueven en el lugar)
            for idx in motor_indices:
                # Alternar fase para patas opuestas
                phase = 0 if idx % 2 == 0 else np.pi
                target = 0.5 * np.sin(2 * np.pi * 0.5 * t + phase)
                p.setJointMotorControl2(robot_id, idx, p.POSITION_CONTROL, targetPosition=target)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            t += 1.0 / 240.0
    except KeyboardInterrupt:
        print("\nSimulación finalizada por el usuario.")

    p.disconnect()

if __name__ == "__main__":
    import numpy as np
    main()
