import numpy as np

# Pesos de la función objetivo (se pueden ajustar más adelante)
W_POS = 1.0   # error de posición (avance en X)
W_LAT = 0.5   # desviación lateral en Y
W_ORI = 0.2   # error de orientación medio
W_TIME = 0.001  # tiempo / número de pasos
P_QUEDA = 100.0  # penalización por caída


def detectar_caida(metrics, z_min=0.15):
    """Detección simple de caída basada en la altura z.

    Si en algún paso la z de la base es menor que z_min,
    consideramos que el robot ha caído.
    """
    # Si no hay z en las métricas, asumimos que no se registró caída
    if len(metrics) == 0 or 'z' not in metrics[0]:
        return 0
    caido = any(m['z'] < z_min for m in metrics)
    return 1 if caido else 0


def funcion_objetivo(metrics,
                     x_objetivo=10.0,
                     w_pos=W_POS,
                     w_lat=W_LAT,
                     w_ori=W_ORI,
                     w_time=W_TIME,
                     p_queda=P_QUEDA):
  
    if not metrics:
        return float('inf')

    # Posición inicial y final
    x0, y0 = metrics[0]['x'], metrics[0]['y']
    xf, yf = metrics[-1]['x'], metrics[-1]['y']

    # Error de avance y desviación lateral
    dx = x_objetivo - xf
    dy = yf - y0

    # Error de orientación medio (si existe en las métricas)
    if 'orientacion' in metrics[0]:
        orientaciones = np.array([abs(m['orientacion']) for m in metrics])
        e_ori_mean = float(np.mean(orientaciones))
    else:
        e_ori_mean = 0.0

    # Tiempo / número de pasos
    T = len(metrics)

    # Detección de caída
    cayo = detectar_caida(metrics)

    J = (
        w_pos * abs(dx)
        + w_lat * abs(dy)
        + w_ori * e_ori_mean
        + w_time * T
        + p_queda * cayo
    )
    return float(J)
