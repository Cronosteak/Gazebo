from laikago_robot import LaikagoRobot
from fuzzy_controller import fuzzy_step
import numpy as np
import time
import pybullet as p
import csv

# Límites de seguridad para los ángulos
MUSLO_MIN, MUSLO_MAX = -2.0, 2.5
RODILLA_MIN, RODILLA_MAX = -3.0, 0.0

# Punto A (inicio) y punto B (objetivo)
PUNTO_A = np.array([0.0, 0.0])
PUNTO_B = np.array([10.0, 0.0])

MAX_FORCE = 65
MAX_VELOCITY = 3.5
GAIT_CYCLE_TIME = 1.2
N_STEPS = 200
ABDUCCION_BASE = 0.0
pata_phase = [0.0, 0.5, 0.5, 0.0]  # FR, FL, RR, RL


def run_simulation(max_steps=2000, gui=False, guardar_csv=True, csv_path="metricas_simulacion.csv"):
    """Ejecuta una simulación completa y devuelve la lista de métricas.

    Esta función es la que debe llamar PSO/GA. Si `guardar_csv` es True,
    también escribe las métricas en un archivo CSV para análisis posterior.
    """

    robot = LaikagoRobot(gui=gui)
    robot.connect()
    robot.reset_pose()

    # Configurar cámara inicial en vista lateral (si hay GUI)
    if robot.gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=2.5,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[0.0, 0.0, 0.5],
        )

    # Estabilizar
    for _ in range(2 * 240):
        robot.step()

    metrics = []
    THRESHOLD_DIST = 0.2  # Distancia mínima al objetivo para terminar

    try:
        for step in range(max_steps):
            pos, orn = robot.get_base_position()
            pos_xy = np.array([pos[0], pos[1]])
            dist_to_goal = np.linalg.norm(PUNTO_B - pos_xy)
            error_avance = PUNTO_B[0] - pos_xy[0]
            ang_obj = np.arctan2(PUNTO_B[1] - pos_xy[1], PUNTO_B[0] - pos_xy[0]) * 180/np.pi
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2] * 180/np.pi
            error_orientacion = ang_obj - yaw

            # Terminar si el robot está suficientemente cerca del objetivo
            if dist_to_goal < THRESHOLD_DIST:
                # print(f"Objetivo alcanzado en el paso {step}. Distancia al objetivo: {dist_to_goal:.3f} m")
                break

            fase_actual = int((step // (N_STEPS // 2)) % 2)
            outputs = fuzzy_step(fase_actual, error_avance)
            avanzar = outputs['cmd_avance'] > 0.3

            muslo = []
            rodilla = []
            # Mapeo de patas: FR=0, FL=1, RR=2, RL=3
            pares_diagonales = [[1, 2], [0, 3]]  # FL+RR, FR+RL
            if avanzar:
                patas_en_avance = pares_diagonales[fase_actual]
            else:
                patas_en_avance = []

            rodilla_fuzzy = [
                outputs['cmd_rodilla_FR'],
                outputs['cmd_rodilla_FL'],
                outputs['cmd_rodilla_RR'],
                outputs['cmd_rodilla_RL'],
            ]

            for i in range(4):
                phase = ((step / N_STEPS) + pata_phase[i]) % 1.0
                rodilla_cmd = rodilla_fuzzy[i]
                # Sincronización temporal: solo permitir avance si la opuesta está apoyada
                opuesta = {0: 1, 1: 0, 2: 3, 3: 2}[i]  # FR<->FL, RR<->RL
                rodilla_opuesta = rodilla_fuzzy[opuesta]
                if i in patas_en_avance and rodilla_opuesta > 0.15:
                    # Si la opuesta no está suficientemente apoyada, forzar rodilla baja
                    rodilla_limited = -0.8
                else:
                    rodilla_limited = max(min(rodilla_cmd, -0.5), -0.8)
                if rodilla_limited < -0.7:
                    muslo_calc = 0.7 * np.sin(2 * np.pi * phase)
                elif rodilla_limited > -0.6:
                    muslo_calc = -0.3 * np.sin(2 * np.pi * phase)
                else:
                    muslo_calc = 0.2 * np.sin(2 * np.pi * phase)
                muslo_limited = min(max(muslo_calc, MUSLO_MIN), MUSLO_MAX)
                muslo.append(muslo_limited)
                rodilla.append(rodilla_limited)

            # Abducción desactivada, se pasa [0.0]*4
            if not avanzar:
                pos, orn = robot.get_base_position()
                new_pos = [pos[0] + 0.01, pos[1], pos[2]]
                p.resetBasePositionAndOrientation(robot.robot, new_pos, orn)
            robot.set_joint_commands([0.0] * 4, muslo, rodilla, max_force=MAX_FORCE, max_velocity=MAX_VELOCITY)
            robot.step()
            pos, _ = robot.get_base_position()

            # Actualizar cámara para vista lateral (solo si GUI)
            if robot.gui:
                p.resetDebugVisualizerCamera(
                    cameraDistance=2.5,
                    cameraYaw=90,
                    cameraPitch=0,
                    cameraTargetPosition=[pos[0], pos[1], 0.5],
                )

            metrics.append({
                'step': step,
                'x': pos[0],
                'y': pos[1],
                'z': pos[2],
                'avance': error_avance,
                'orientacion': error_orientacion,
                'cmd_avance': outputs['cmd_avance'],
                'cmd_FL': outputs['cmd_FL'],
                'cmd_FR': outputs['cmd_FR'],
                'cmd_RR': outputs['cmd_RR'],
                'cmd_RL': outputs['cmd_RL'],
            })

            time.sleep(GAIT_CYCLE_TIME / N_STEPS)

    finally:
        # Guardar CSV si se solicita
        if guardar_csv and len(metrics) > 0:
            with open(csv_path, 'w', newline='') as csvfile:
                fieldnames = ['step', 'x', 'y', 'z', 'avance', 'orientacion', 'cmd_avance', 'cmd_FL', 'cmd_FR', 'cmd_RR', 'cmd_RL']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for row in metrics:
                    writer.writerow(row)

        robot.disconnect()
        return metrics


if __name__ == "__main__":
    # Permite ejecutar la simulación manualmente desde la terminal
    print("Iniciando simulación standalone...")
    run_simulation(max_steps=2000, gui=True, guardar_csv=True)
