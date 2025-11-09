from laikago_robot import LaikagoRobot
from fuzzy_controller import fuzzy_step
import numpy as np
import time
import pybullet as p

# Punto A (inicio) y punto B (objetivo)
PUNTO_A = np.array([0.0, 0.0])
PUNTO_B = np.array([10.0, 0.0])

robot = LaikagoRobot(gui=True)
robot.connect()
robot.reset_pose()

# Esperar 5 segundos para que el robot se estabilice en el piso
print("Esperando 5 segundos para estabilizar la pose inicial...")
for _ in range(5 * 240):  # 5 segundos a 240Hz
    robot.step()
print("Inicio del ciclo de marcha.")

MAX_FORCE = 80
MAX_VELOCITY = 2.0

# Parámetros del ciclo de marcha suave ajustados
GAIT_CYCLE_TIME = 1.2  # segundos por ciclo completo
N_STEPS = 200          # pasos por ciclo
AMPLITUD_MUSLO = 0.35  # amplitud mayor para levantar y avanzar
AMPLITUD_RODILLA = 0.3
MUSLO_BASE = 0.25      # posición base cerca del mínimo
RODILLA_BASE = -2.5    # rodilla base cerca del mínimo
ABDUCCION_BASE = 0.0
ABDUCCION_LIFT = 0.12  # abducción para pata en avance

# Mapeo de patas: [FR, FL, RR, RL]
pata_phase = [0.0, 0.5, 0.25, 0.75]  # FR, FL, RR, RL (delanteras desfasadas respecto a traseras)

# Límites de seguridad para los ángulos
MUSLO_MIN, MUSLO_MAX = -2.0, 2.5
RODILLA_MIN, RODILLA_MAX = -3.0, 0.0

step = 0

while True:
    pos, orn = robot.get_base_position()
    pos_xy = np.array([pos[0], pos[1]])
    error_avance = PUNTO_B[0] - pos_xy[0]
    ang_obj = np.arctan2(PUNTO_B[1] - pos_xy[1], PUNTO_B[0] - pos_xy[0]) * 180/np.pi
    import pybullet as p
    euler = p.getEulerFromQuaternion(orn)
    yaw = euler[2] * 180/np.pi
    error_orientacion = ang_obj - yaw

    # Fuzzy decide si avanzar
    fase_actual = int((step // (N_STEPS // 4)) % 4)
    outputs = fuzzy_step(fase_actual, error_avance, error_orientacion)
    #print(f"Step: {step} | Fase: {fase_actual}")
    #print(f"Comandos fuzzy: avance={outputs['cmd_avance']:.2f}, FL={outputs['cmd_FL']:.2f}, FR={outputs['cmd_FR']:.2f}, RR={outputs['cmd_RR']:.2f}, RL={outputs['cmd_RL']:.2f}")
    avanzar = outputs['cmd_avance'] > 0.3


    muslo = []
    rodilla = []
    abduccion = []
    # Secuencia: FL → RR → FR → RL (solo una pata avanza por fase)
    orden_patas = [1, 2, 0, 3]  # FL, RR, FR, RL
    if avanzar:
        pata_en_avance = orden_patas[fase_actual % 4]
    else:
        pata_en_avance = None

    # Parámetros de amplitud para paso realista
    AMPLITUD_MUSLO_REAL = 2.0
    AMPLITUD_RODILLA_REAL = 2.5

    for i in range(4):
        phase = ((step / N_STEPS) + pata_phase[i]) % 1.0
        # Hombros (abducción) se mueven de forma sinusoidal en todo el ciclo
        abduccion_i = ABDUCCION_BASE + ABDUCCION_LIFT * np.sin(2 * np.pi * phase)
        if avanzar and i == pata_en_avance:
            # Paso largo: muslo adelante, rodilla extendida
            muslo_calc = 1.0 + 0.8 * np.sin(2 * np.pi * phase)
            rodilla_calc = -1.2 + 0.5 * np.abs(np.cos(2 * np.pi * phase))
            abduccion_i += ABDUCCION_LIFT  # Aumenta amplitud en avance
        else:
            muslo_calc = 1.0
            rodilla_calc = -1.2
        muslo_limited = min(max(muslo_calc, MUSLO_MIN), MUSLO_MAX + AMPLITUD_MUSLO_REAL)
        rodilla_limited = min(max(rodilla_calc, RODILLA_MIN), RODILLA_MAX)
        abduccion_limited = min(max(abduccion_i, -0.5), 0.5)
        muslo.append(muslo_limited)
        rodilla.append(rodilla_limited)
        abduccion.append(abduccion_limited)
        print(f"[DEBUG] Pata {i}: muslo_calc={muslo_calc:.3f}, muslo_limited={muslo_limited:.3f}, rodilla_calc={rodilla_calc:.3f}, rodilla_limited={rodilla_limited:.3f}, abduccion={abduccion_limited:.3f}")
        muslo_limited = min(max(muslo_calc, MUSLO_MIN), MUSLO_MAX + AMPLITUD_MUSLO_REAL)
        rodilla_limited = min(max(rodilla_calc, RODILLA_MIN), RODILLA_MAX)
        muslo.append(muslo_limited)
        rodilla.append(rodilla_limited)
        abduccion.append(abduccion_i)
        print(f"[DEBUG] Pata {i}: muslo_calc={muslo_calc:.3f}, muslo_limited={muslo_limited:.3f}, rodilla_calc={rodilla_calc:.3f}, rodilla_limited={rodilla_limited:.3f}")
    # Controlador de avance del tronco: si la pata está en avance, el cuerpo se mantiene atrás
    # Solo se desplaza cuando las patas están en apoyo
    if not avanzar:
        pos, orn = robot.get_base_position()
        new_pos = [pos[0] + 0.01, pos[1], pos[2]]
        p.resetBasePositionAndOrientation(robot.robot, new_pos, orn)
    #print(f"Ángulos muslo: {muslo}")
    #print(f"Ángulos rodilla: {rodilla}")
    #print(f"Ángulos abducción: {abduccion}")
    robot.set_joint_commands(abduccion, muslo, rodilla, max_force=120, max_velocity=MAX_VELOCITY)
    robot.step()
    pos, _ = robot.get_base_position()
    #print(f"Posición base: {pos}")
    step += 1
    time.sleep(GAIT_CYCLE_TIME / N_STEPS)
