import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Variables de entrada
fase = ctrl.Antecedent(np.arange(0, 4, 1), 'fase')  # 0:FL, 1:RR, 2:FR, 3:RL
error_avance = ctrl.Antecedent(np.arange(-5, 6, 1), 'error_avance')
error_orientacion = ctrl.Antecedent(np.arange(-20, 21, 1), 'error_orientacion')


# Variables de salida (comando para cada pata)
cmd_FR = ctrl.Consequent(np.arange(0, 2, 1), 'cmd_FR')  # 0: no mover, 1: mover
cmd_FL = ctrl.Consequent(np.arange(0, 2, 1), 'cmd_FL')
cmd_RR = ctrl.Consequent(np.arange(0, 2, 1), 'cmd_RR')
cmd_RL = ctrl.Consequent(np.arange(0, 2, 1), 'cmd_RL')
cmd_avance = ctrl.Consequent(np.arange(0, 2, 1), 'cmd_avance')  # 0: no avanzar, 1: avanzar

# Funciones de pertenencia para fase
fase['FL'] = fuzz.trimf(fase.universe, [0, 0, 0.5])
fase['RR'] = fuzz.trimf(fase.universe, [0.5, 1, 1.5])
fase['FR'] = fuzz.trimf(fase.universe, [1.5, 2, 2.5])
fase['RL'] = fuzz.trimf(fase.universe, [2.5, 3, 3])

# Funciones de pertenencia para error de avance
error_avance['neg'] = fuzz.trimf(error_avance.universe, [-5, -5, 0])
error_avance['cero'] = fuzz.trimf(error_avance.universe, [-2, 0, 2])
error_avance['pos'] = fuzz.trimf(error_avance.universe, [0, 5, 5])

# Funciones de pertenencia para error de orientación
error_orientacion['izq'] = fuzz.trimf(error_orientacion.universe, [-20, -20, 0])
error_orientacion['centro'] = fuzz.trimf(error_orientacion.universe, [-10, 0, 10])
error_orientacion['der'] = fuzz.trimf(error_orientacion.universe, [0, 20, 20])

# Funciones de pertenencia para comandos
for cmd in [cmd_FL, cmd_RR, cmd_FR, cmd_RL, cmd_avance]:
    cmd['no'] = fuzz.trimf(cmd.universe, [0, 0, 1])
    cmd['si'] = fuzz.trimf(cmd.universe, [0, 1, 1])



# Reglas de avance secuencial por fase
rules = [
    ctrl.Rule(fase['FL'] & error_avance['pos'], cmd_FL['si']),
    ctrl.Rule(fase['FL'] & error_avance['pos'], cmd_avance['si']),
    ctrl.Rule(fase['RR'] & error_avance['pos'], cmd_RR['si']),
    ctrl.Rule(fase['RR'] & error_avance['pos'], cmd_avance['si']),
    ctrl.Rule(fase['FR'] & error_avance['pos'], cmd_FR['si']),
    ctrl.Rule(fase['FR'] & error_avance['pos'], cmd_avance['si']),
    ctrl.Rule(fase['RL'] & error_avance['pos'], cmd_RL['si']),
    ctrl.Rule(fase['RL'] & error_avance['pos'], cmd_avance['si']),
    # Mantener equilibrio según orientación
    ctrl.Rule(error_orientacion['izq'], cmd_FL['no']),
    ctrl.Rule(error_orientacion['der'], cmd_FR['no']),
    # Reglas por defecto para asegurar que todos los comandos tengan valor
    ctrl.Rule(~fase['FR'], cmd_FR['no']),
    ctrl.Rule(~fase['FL'], cmd_FL['no']),
    ctrl.Rule(~fase['RR'], cmd_RR['no']),
    ctrl.Rule(~fase['RL'], cmd_RL['no']),
    ctrl.Rule(error_avance['cero'], cmd_avance['no']),
    # Regla: avanzar si al menos dos patas están en apoyo (trote)
    ctrl.Rule(
        ((cmd_FR['si'] & cmd_RL['si']) |
         (cmd_FL['si'] & cmd_RR['si']) |
         (cmd_FR['si'] & cmd_FL['si']) |
         (cmd_RR['si'] & cmd_RL['si'])),
        cmd_avance['si']
    ),
]

# Sistema de control
fuzzy_ctrl = ctrl.ControlSystem(rules)
fuzzy_sim = ctrl.ControlSystemSimulation(fuzzy_ctrl)

# Función para obtener comandos fuzzy

def fuzzy_step(fase_val, error_avance_val, error_orientacion_val):
    fuzzy_sim.input['fase'] = fase_val
    fuzzy_sim.input['error_avance'] = error_avance_val
    fuzzy_sim.input['error_orientacion'] = error_orientacion_val
    fuzzy_sim.compute()
    # Asegurar que todas las salidas estén presentes
    outputs = {}
    for key in ['cmd_FR', 'cmd_FL', 'cmd_RR', 'cmd_RL', 'cmd_avance']:
        outputs[key] = fuzzy_sim.output.get(key, 0.0)
    return outputs
