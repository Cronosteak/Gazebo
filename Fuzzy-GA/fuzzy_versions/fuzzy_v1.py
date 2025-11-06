# frozen version of fuzzy_controller.py (v1)
# Fecha: 2025-10-30
# Esta versión corresponde al sistema base antes de optimización GA.

import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt

def crear_fuzzy_controller(
    pos_centros=(0, 2, 5), pos_anchos=(1, 1.5, 1),
    pitch_centros=(-20, 0, 20), pitch_anchos=(10, 10, 10),
    salida_centros=(-1.5, 0, 1.5), salida_anchos=(0.5, 0.5, 0.5)
):
    error_pos = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'error_pos')
    error_pitch = ctrl.Antecedent(np.arange(-20, 21, 1), 'error_pitch')
    ajuste_articular = ctrl.Consequent(np.arange(-1.5, 1.6, 0.1), 'ajuste_articular')
    error_pos['cerca'] = fuzz.trimf(error_pos.universe, [pos_centros[0]-pos_anchos[0], pos_centros[0], pos_centros[0]+pos_anchos[0]])
    error_pos['medio'] = fuzz.trimf(error_pos.universe, [pos_centros[1]-pos_anchos[1], pos_centros[1], pos_centros[1]+pos_anchos[1]])
    error_pos['lejos'] = fuzz.trimf(error_pos.universe, [pos_centros[2]-pos_anchos[2], pos_centros[2], pos_centros[2]+pos_anchos[2]])
    error_pitch['negativo'] = fuzz.trimf(error_pitch.universe, [pitch_centros[0]-pitch_anchos[0], pitch_centros[0], pitch_centros[0]+pitch_anchos[0]])
    error_pitch['cero'] = fuzz.trimf(error_pitch.universe, [pitch_centros[1]-pitch_anchos[1], pitch_centros[1], pitch_centros[1]+pitch_anchos[1]])
    error_pitch['positivo'] = fuzz.trimf(error_pitch.universe, [pitch_centros[2]-pitch_anchos[2], pitch_centros[2], pitch_centros[2]+pitch_anchos[2]])
    ajuste_articular['bajar'] = fuzz.trimf(ajuste_articular.universe, [salida_centros[0]-salida_anchos[0], salida_centros[0], salida_centros[0]+salida_anchos[0]])
    ajuste_articular['nada'] = fuzz.trimf(ajuste_articular.universe, [salida_centros[1]-salida_anchos[1], salida_centros[1], salida_centros[1]+salida_anchos[1]])
    ajuste_articular['subir'] = fuzz.trimf(ajuste_articular.universe, [salida_centros[2]-salida_anchos[2], salida_centros[2], salida_centros[2]+salida_anchos[2]])
    regla1 = ctrl.Rule(error_pos['lejos'] | error_pitch['negativo'], ajuste_articular['subir'])
    regla2 = ctrl.Rule(error_pos['cerca'] & error_pitch['cero'], ajuste_articular['nada'])
    regla3 = ctrl.Rule(error_pos['medio'] | error_pitch['positivo'], ajuste_articular['bajar'])
    sistema_ctrl = ctrl.ControlSystem([regla1, regla2, regla3])
    return ctrl.ControlSystemSimulation(sistema_ctrl)

def calcular_ajuste(error_pos_val, error_pitch_val, fuzzy_ctrl=None):
    if fuzzy_ctrl is None:
        ctrl_sim = crear_fuzzy_controller()
    else:
        ctrl_sim = fuzzy_ctrl
    ctrl_sim.input['error_pos'] = error_pos_val
    ctrl_sim.input['error_pitch'] = error_pitch_val
    ctrl_sim.compute()
    return ctrl_sim.output['ajuste_articular']

# No incluye visualización para mantenerlo congelado y portable.
