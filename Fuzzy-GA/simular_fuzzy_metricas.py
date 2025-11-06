import pybullet as p
import pybullet_data
import numpy as np
import time
import csv
from fuzzy_controller import fuzzy_avance, fuzzy_direccion

# Configuración de simulación
DT = 0.01

# Inicializar PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# Cargar plano y robot (ajusta el path de URDF según tu entorno)
plane = p.loadURDF("plane.urdf")
horizontal_orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])
robot = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, 0.45], baseOrientation=horizontal_orientation)


# IDs de las articulaciones relevantes (ajusta según tu robot)
JOINT_NAMES = ['FL_hip', 'FL_thigh', 'FL_calf', 'FR_hip', 'FR_thigh', 'FR_calf',
			   'RL_hip', 'RL_thigh', 'RL_calf', 'RR_hip', 'RR_thigh', 'RR_calf']
JOINT_IDS = list(range(12))

# Parámetros de marcha crawl (patrón cruzado)
def crawl_pattern(step, amp=0.3):
	phase = (step * DT * 2 * np.pi) / 1.0
	# FL+RR avanzan, FR+RL retroceden
	muslo = [amp * np.sin(phase), amp * np.sin(phase + np.pi),
			 amp * np.sin(phase + np.pi), amp * np.sin(phase)]
	rodilla = [amp * np.cos(phase), amp * np.cos(phase + np.pi),
			   amp * np.cos(phase + np.pi), amp * np.cos(phase)]
	# Compensación de cadera/abducción para estabilidad
	abduccion = [0.1, -0.1, -0.1, 0.1]  # FL, FR, RL, RR
	return muslo, rodilla, abduccion

# Registro de métricas
metrics = []

step = 0
while p.isConnected():
	# Leer posición y orientación base
	pos, orn = p.getBasePositionAndOrientation(robot)
	# Errores para fuzzy (ejemplo: distancia al objetivo, orientación)
	error_avance = 0.0  # Aquí puedes calcular el error real
	error_direccion = 0.0

	# Fase inicial: robot estable (primeros 2 segundos)
	if step < int(2.0 / DT):
		avance_cmd = 0.0
		direccion_cmd = 0.0
	else:
		avance_cmd = fuzzy_avance(5.0, error_avance)  # Ejemplo: avanzar
		direccion_cmd = fuzzy_direccion(0.0, error_direccion)  # Ejemplo: sin giro

	# Patrón de marcha crawl con compensación
	muslo, rodilla, abduccion = crawl_pattern(step, amp=avance_cmd)

	# Parámetros de control
	max_force = 40  # Fuerza máxima intermedia
	max_velocity = 2.0  # Velocidad máxima intermedia
	for i, jid in enumerate(JOINT_IDS):
		if 'hip' in JOINT_NAMES[i]:
			p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=abduccion[i//3], force=max_force, maxVelocity=max_velocity)
		elif 'thigh' in JOINT_NAMES[i]:
			p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=muslo[i//3], force=max_force, maxVelocity=max_velocity)
		elif 'calf' in JOINT_NAMES[i]:
			p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=rodilla[i//3], force=max_force, maxVelocity=max_velocity)
		else:
			p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=0.0, force=max_force, maxVelocity=max_velocity)

	# Guardar métricas
	metrics.append({
		'step': step,
		'x': pos[0],
		'y': pos[1],
		'z': pos[2],
		'avance_cmd': avance_cmd,
		'direccion_cmd': direccion_cmd
	})

	p.stepSimulation()
	time.sleep(DT)
	step += 1

# Guardar métricas en CSV
with open('metricas_simulacion.csv', 'w', newline='') as csvfile:
	fieldnames = ['step', 'x', 'y', 'z', 'avance_cmd', 'direccion_cmd']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	for row in metrics:
		writer.writerow(row)

p.disconnect()
