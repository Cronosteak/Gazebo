import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Controlador fuzzy para avance/retroceso
avance = ctrl.Antecedent(np.arange(-10, 11, 1), 'avance')
error_avance = ctrl.Antecedent(np.arange(-5, 6, 1), 'error_avance')
salida_avance = ctrl.Consequent(np.arange(-1, 1.1, 0.1), 'salida_avance')

avance['retroceso'] = fuzz.trimf(avance.universe, [-10, -10, 0])
avance['neutro'] = fuzz.trimf(avance.universe, [-5, 0, 5])
avance['avance'] = fuzz.trimf(avance.universe, [0, 10, 10])

error_avance['neg'] = fuzz.trimf(error_avance.universe, [-5, -5, 0])
error_avance['cero'] = fuzz.trimf(error_avance.universe, [-2, 0, 2])
error_avance['pos'] = fuzz.trimf(error_avance.universe, [0, 5, 5])

salida_avance['retroceso'] = fuzz.trimf(salida_avance.universe, [-1, -1, 0])
salida_avance['neutro'] = fuzz.trimf(salida_avance.universe, [-0.5, 0, 0.5])
salida_avance['avance'] = fuzz.trimf(salida_avance.universe, [0, 1, 1])

rules_avance = [
	ctrl.Rule(avance['avance'] & error_avance['cero'], salida_avance['avance']),
	ctrl.Rule(avance['retroceso'] & error_avance['cero'], salida_avance['retroceso']),
	ctrl.Rule(avance['neutro'] & error_avance['cero'], salida_avance['neutro']),
	ctrl.Rule(error_avance['neg'], salida_avance['retroceso']),
	ctrl.Rule(error_avance['pos'], salida_avance['avance']),
	# Nueva regla: si el error de avance es muy negativo (robot se agacha mucho), levantar a 60°
	ctrl.Rule(error_avance['neg'] & avance['neutro'], salida_avance['avance'])
]

avance_ctrl = ctrl.ControlSystem(rules_avance)
avance_sim = ctrl.ControlSystemSimulation(avance_ctrl)

# Controlador fuzzy para dirección lateral
direccion = ctrl.Antecedent(np.arange(-10, 11, 1), 'direccion')
error_direccion = ctrl.Antecedent(np.arange(-5, 6, 1), 'error_direccion')
salida_direccion = ctrl.Consequent(np.arange(-1, 1.1, 0.1), 'salida_direccion')

direccion['izq'] = fuzz.trimf(direccion.universe, [-10, -10, 0])
direccion['centro'] = fuzz.trimf(direccion.universe, [-5, 0, 5])
direccion['der'] = fuzz.trimf(direccion.universe, [0, 10, 10])

error_direccion['neg'] = fuzz.trimf(error_direccion.universe, [-5, -5, 0])
error_direccion['cero'] = fuzz.trimf(error_direccion.universe, [-2, 0, 2])
error_direccion['pos'] = fuzz.trimf(error_direccion.universe, [0, 5, 5])

salida_direccion['izq'] = fuzz.trimf(salida_direccion.universe, [-1, -1, 0])
salida_direccion['centro'] = fuzz.trimf(salida_direccion.universe, [-0.5, 0, 0.5])
salida_direccion['der'] = fuzz.trimf(salida_direccion.universe, [0, 1, 1])

rules_direccion = [
	ctrl.Rule(direccion['izq'] & error_direccion['cero'], salida_direccion['izq']),
	ctrl.Rule(direccion['der'] & error_direccion['cero'], salida_direccion['der']),
	ctrl.Rule(direccion['centro'] & error_direccion['cero'], salida_direccion['centro']),
	ctrl.Rule(error_direccion['neg'], salida_direccion['izq']),
	ctrl.Rule(error_direccion['pos'], salida_direccion['der']),
	# Nueva regla: si el error de dirección es muy negativo (ej. pitch muy negativo), corregir hacia el centro
	ctrl.Rule(error_direccion['neg'] & direccion['centro'], salida_direccion['centro'])
]

direccion_ctrl = ctrl.ControlSystem(rules_direccion)
direccion_sim = ctrl.ControlSystemSimulation(direccion_ctrl)

def fuzzy_avance(avance_val, error_val):
	avance_sim.input['avance'] = avance_val
	avance_sim.input['error_avance'] = error_val
	avance_sim.compute()
	return avance_sim.output['salida_avance']

def fuzzy_direccion(dir_val, error_val):
	direccion_sim.input['direccion'] = dir_val
	direccion_sim.input['error_direccion'] = error_val
	direccion_sim.compute()
	return direccion_sim.output['salida_direccion']
