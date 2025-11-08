

from laikago_robot import LaikagoRobot
from fuzzy_controller import fuzzy_step
import numpy as np

# Punto A (inicio) y punto B (objetivo)
PUNTO_A = np.array([0.0, 0.0])
PUNTO_B = np.array([1.0, 0.0])

robot = LaikagoRobot(gui=True)
robot.connect()
robot.reset_pose()

secuencia_patas = [0, 1, 2, 3]  # FR, FL, RR, RL (índices)
nombre_patas = ['cmd_FR', 'cmd_FL', 'cmd_RR', 'cmd_RL']
MAX_FORCE = 100
MAX_VELOCITY = 4.0
SECUENCIA = [
    (1.2, -0.6),   # levantar
    (1.5, -0.8),   # adelantar
    (0.67, -1.25)  # apoyar
]

paso = 0
fase = 0

while True:
    pos, orn = robot.get_base_position()
    pos_xy = np.array([pos[0], pos[1]])
    error_avance = PUNTO_B[0] - pos_xy[0]
    ang_obj = np.arctan2(PUNTO_B[1] - pos_xy[1], PUNTO_B[0] - pos_xy[0]) * 180/np.pi
    import pybullet as p
    euler = p.getEulerFromQuaternion(orn)
    yaw = euler[2] * 180/np.pi
    error_orientacion = ang_obj - yaw

    outputs = fuzzy_step(fase, error_avance, error_orientacion)
    print(f"Fase: {fase} | Paso: {paso}")
    print(f"Comandos difusos: {outputs}")
    print(f"Posición base: {pos_xy}")


    abduccion = [0.0]*4
    muslo = [0.67]*4
    rodilla = [-1.25]*4


    # Secuencia: mueve una pata por ciclo y empuja el cuerpo
    if outputs['cmd_avance'] > 0.3 and outputs[nombre_patas[secuencia_patas[fase]]] > 0.3:
        print(f"Moviendo pata: {nombre_patas[secuencia_patas[fase]]} (índice {secuencia_patas[fase]})")
        muslo_avance, rodilla_avance = SECUENCIA[paso]
        pata = secuencia_patas[fase]
        muslo[pata] = muslo_avance + 0.3
        rodilla[pata] = rodilla_avance + 0.2
        abduccion[pata] = 0.3  # abrir lateralmente la pata que avanza
        # Empuje del cuerpo: mueve ligeramente todas las patas hacia adelante
        for i in range(4):
            if i != pata:
                muslo[i] += 0.15

        # Forzar pequeño desplazamiento del cuerpo
        pos, orn = robot.get_base_position()
        nueva_pos = [pos[0] + 0.01, pos[1], pos[2]]
        p.resetBasePositionAndOrientation(robot.robot, nueva_pos, orn)

    # Las patas en soporte se mantienen en pose inicial
    for i, key in enumerate(nombre_patas):
        if outputs[key] < 0.5:
            muslo[i] = 0.67
            rodilla[i] = -1.25

    robot.set_joint_commands(abduccion, muslo, rodilla, max_force=MAX_FORCE, max_velocity=MAX_VELOCITY)
    robot.step()

    # Avanzar secuencia y fase
    paso = (paso + 1) % len(SECUENCIA)
    if paso == 0:
        fase = (fase + 1) % 4
