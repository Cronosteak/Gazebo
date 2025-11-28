import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


def _crear_sistema_fuzzy(parametros=None):
    """Crea el sistema difuso.

    Si `parametros` es None, usa los valores por defecto.
    De lo contrario, espera un vector:

    parametros = [
        c_neg,          # centro de error_avance['neg']
        c_cero_izq,     # extremo izq de error_avance['cero']
        c_cero_der,     # extremo der de error_avance['cero']
        c_pos,          # centro de error_avance['pos']
        a_neg_der,      # extremo der de cmd_avance['neg']
        a_cero_izq,     # extremo izq de cmd_avance['cero']
        a_cero_der,     # extremo der de cmd_avance['cero']
        a_pos_izq       # extremo izq de cmd_avance['pos']
    ]
    """

    # Variables de entrada mínimas
    fase = ctrl.Antecedent(np.arange(0, 2, 1), 'fase')  # 0:FL+RR, 1:FR+RL
    error_avance = ctrl.Antecedent(np.arange(-5, 6, 1), 'error_avance')

    # Variables de salida mínimas
    cmd_FR = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_FR')
    cmd_FL = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_FL')
    cmd_RR = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_RR')
    cmd_RL = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_RL')
    cmd_avance = ctrl.Consequent(np.arange(-1, 1.1, 0.1), 'cmd_avance')
    cmd_rodilla_FL = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_rodilla_FL')
    cmd_rodilla_FR = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_rodilla_FR')
    cmd_rodilla_RR = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_rodilla_RR')
    cmd_rodilla_RL = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'cmd_rodilla_RL')

    # Funciones de pertenencia de fase (fijas)
    fase['pares1'] = fuzz.trimf(fase.universe, [0, 0, 1])
    fase['pares2'] = fuzz.trimf(fase.universe, [0, 1, 1])

    # Parámetros por defecto para error_avance y cmd_avance
    if parametros is None:
        c_neg = 0.0
        c_cero_izq, c_cero_der = -2.0, 2.0
        c_pos = 0.0
        a_neg_der = 0.0
        a_cero_izq, a_cero_der = -0.5, 0.5
        a_pos_izq = 0.0
    else:
        (
            c_neg,
            c_cero_izq,
            c_cero_der,
            c_pos,
            a_neg_der,
            a_cero_izq,
            a_cero_der,
            a_pos_izq,
        ) = parametros

    # Asegurar orden lógico dentro de cotas razonables
    c_neg = np.clip(c_neg, -5.0, 0.0)
    c_cero_izq = np.clip(c_cero_izq, -5.0, 0.0)
    c_cero_der = np.clip(c_cero_der, 0.0, 5.0)
    c_pos = np.clip(c_pos, 0.0, 5.0)

    a_neg_der = np.clip(a_neg_der, -1.0, 0.0)
    a_cero_izq = np.clip(a_cero_izq, -1.0, 0.0)
    a_cero_der = np.clip(a_cero_der, 0.0, 1.0)
    a_pos_izq = np.clip(a_pos_izq, 0.0, 1.0)

    # Funciones de pertenencia para error_avance
    error_avance['neg'] = fuzz.trimf(error_avance.universe, [-5, -5, c_neg])
    error_avance['cero'] = fuzz.trimf(error_avance.universe, [c_cero_izq, 0, c_cero_der])
    error_avance['pos'] = fuzz.trimf(error_avance.universe, [c_pos, 5, 5])

    # Funciones de pertenencia para salidas
    for cmd in [cmd_FR, cmd_FL, cmd_RR, cmd_RL]:
        cmd['no'] = fuzz.trimf(cmd.universe, [0, 0, 0.5])
        cmd['si'] = fuzz.trimf(cmd.universe, [0.5, 1, 1])

    cmd_avance['neg'] = fuzz.trimf(cmd_avance.universe, [-1, -1, a_neg_der])
    cmd_avance['cero'] = fuzz.trimf(cmd_avance.universe, [a_cero_izq, 0, a_cero_der])
    cmd_avance['pos'] = fuzz.trimf(cmd_avance.universe, [a_pos_izq, 1, 1])

    for cmd in [cmd_rodilla_FL, cmd_rodilla_FR, cmd_rodilla_RR, cmd_rodilla_RL]:
        cmd['baja'] = fuzz.trimf(cmd.universe, [0, 0, 0.1])
        cmd['alta'] = fuzz.trimf(cmd.universe, [0.8, 1, 1])

    # Reglas mínimas para ciclo de marcha
    rules = [
    # Fase 0: avanzan FL y RR
    ctrl.Rule(fase['pares1'] & error_avance['pos'], cmd_FL['si']),
    ctrl.Rule(fase['pares1'] & error_avance['pos'], cmd_RR['si']),
    ctrl.Rule(fase['pares1'], cmd_rodilla_FL['alta']),
    ctrl.Rule(fase['pares1'], cmd_rodilla_RR['alta']),
    ctrl.Rule(fase['pares1'], cmd_FR['no']),
    ctrl.Rule(fase['pares1'], cmd_RL['no']),
    ctrl.Rule(fase['pares1'], cmd_rodilla_FR['baja']),
    ctrl.Rule(fase['pares1'], cmd_rodilla_RL['baja']),
    # Fase 1: avanzan FR y RL
    ctrl.Rule(fase['pares2'] & error_avance['pos'], cmd_FR['si']),
    ctrl.Rule(fase['pares2'] & error_avance['pos'], cmd_RL['si']),
    ctrl.Rule(fase['pares2'], cmd_rodilla_FR['alta']),
    ctrl.Rule(fase['pares2'], cmd_rodilla_RL['alta']),
    ctrl.Rule(fase['pares2'], cmd_FL['no']),
    ctrl.Rule(fase['pares2'], cmd_RR['no']),
    ctrl.Rule(fase['pares2'], cmd_rodilla_FL['baja']),
    ctrl.Rule(fase['pares2'], cmd_rodilla_RR['baja']),
    # Avance global
    ctrl.Rule(error_avance['neg'], cmd_avance['neg']),
    ctrl.Rule(error_avance['cero'], cmd_avance['cero']),
    ctrl.Rule(error_avance['pos'], cmd_avance['pos']),
    ]

    system = ctrl.ControlSystem(rules)
    fuzzy_sim = ctrl.ControlSystemSimulation(system)
    return system, fuzzy_sim


# Sistema por defecto (sin optimización)
_system, _fuzzy_sim = _crear_sistema_fuzzy()


def set_parametros_fuzzy(parametros):
    """Actualiza las funciones de pertenencia de error_avance y cmd_avance.

    `parametros` debe ser un vector de 8 elementos tal como se documenta
    en `_crear_sistema_fuzzy`. Esta función recrea el sistema y la
    simulación global.
    """
    global _system, _fuzzy_sim
    _system, _fuzzy_sim = _crear_sistema_fuzzy(parametros)


def fuzzy_step(fase_val, error_avance_val):
    """Evalúa un paso del controlador fuzzy con los parámetros actuales."""
    _fuzzy_sim.input['fase'] = fase_val
    _fuzzy_sim.input['error_avance'] = error_avance_val
    _fuzzy_sim.compute()
    outputs = {}
    for key in [
        'cmd_FR', 'cmd_FL', 'cmd_RR', 'cmd_RL', 'cmd_avance',
        'cmd_rodilla_FL', 'cmd_rodilla_FR', 'cmd_rodilla_RR', 'cmd_rodilla_RL',
    ]:
        outputs[key] = _fuzzy_sim.output.get(key, 0.0)
    return outputs
