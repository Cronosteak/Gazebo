import numpy as np

def funcion_objetivo(metrics, penalizacion_caida=100.0):
    """
    Calcula el puntaje de desempeño del robot en la simulación.
    Args:
        metrics (list of dict): Métricas registradas en la simulación.
        penalizacion_caida (float): Penalización si el robot cae.
    Returns:
        float: Puntaje objetivo (mayor es mejor).
    """
    # Distancia recorrida en X
    distancia = metrics[-1]['x'] - metrics[0]['x']
    # Penalización por caída (si z < umbral en algún paso)
    caido = any(m['z'] < 0.2 for m in metrics)
    penalizacion = penalizacion_caida if caido else 0.0
    # Tiempo total de simulación
    tiempo = len(metrics)
    # Puntaje: distancia menos penalización
    puntaje = distancia - penalizacion
    return puntaje
