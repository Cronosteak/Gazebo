import random

import numpy as np
from deap import base, creator, tools

from fuzzy_controller import set_parametros_fuzzy
from funcion_objetivo import funcion_objetivo
from simular_fuzzy_metricas import run_simulation


# Definición de fitness (maximizar) e individuo
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)


toolbox = base.Toolbox()

# Rango de cada uno de los 8 parámetros
# [c_neg, c_cero_izq, c_cero_der, c_pos, a_neg_der, a_cero_izq, a_cero_der, a_pos_izq]
PARAM_MIN = [-10.0, -2.0, 0.0, 0.0, -1.0, -0.5, 0.0, 0.0]
PARAM_MAX = [0.0, 0.0, 2.0, 10.0, 0.0, 0.0, 0.5, 1.0]


def random_param(i: int) -> float:
    return random.uniform(PARAM_MIN[i], PARAM_MAX[i])


def init_individual() -> creator.Individual:
    params = [random_param(i) for i in range(8)]
    return creator.Individual(params)


toolbox.register("individual", init_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)


def objective_function(individual):
    """Función objetivo para DEAP.

    individual: lista de 8 parámetros fuzzy.
    Devuelve una tupla (fitness,) donde fitness = -J (J es el costo).
    """

    # Asegurar tipo lista simple (por si DEAP usa otro contenedor compatible)
    params = list(individual)

    set_parametros_fuzzy(params)

    metrics = run_simulation(max_steps=800, gui=False, guardar_csv=False)

    # Calcular costo J
    J = funcion_objetivo(metrics)

    if (not metrics) or not np.isfinite(J):
        return (-1e9,)

    # DEAP espera una tupla
    return (-J,)


toolbox.register("evaluate", objective_function)
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0.0, sigma=0.3, indpb=0.3)
toolbox.register("select", tools.selTournament, tournsize=3)


def ga_optimize(pop_size=20, n_gen=20, cx_prob=0.7, mut_prob=0.3):
    """Ejecuta un GA simple para optimizar los 8 parámetros fuzzy.

    pop_size y n_gen se dejan pequeños porque cada evaluación implica una
    simulación en PyBullet.
    """

    # Población inicial
    pop = toolbox.population(n=pop_size)

    # Evaluación inicial
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    # Inicializar lista para convergencia
    convergencia = []  # [(gen, max, mean)]

    for gen in range(n_gen):
        print(f"Generación {gen}")

        # Selección y clonación
        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        # Cruce
        for c1, c2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < cx_prob:
                toolbox.mate(c1, c2)
                del c1.fitness.values, c2.fitness.values

        # Mutación
        for mut_ind in offspring:
            if random.random() < mut_prob:
                toolbox.mutate(mut_ind)
                del mut_ind.fitness.values

        # Reevaluar individuos con fitness inválido
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        print(f"  Reevaluando {len(invalid_ind)} individuos...")
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring

        # Fitness máximo y medio de la generación (recordar: fitness = -J)
        fits = [ind.fitness.values[0] for ind in pop]
        max_fit = max(fits)
        mean_fit = sum(fits) / len(fits)
        convergencia.append((gen, max_fit, mean_fit))

        # Mejor individuo de la generación
        best = tools.selBest(pop, 1)[0]
        print(f"  Mejor fitness: {best.fitness.values[0]:.4f}")
        print(f"  Mejor individuo: {list(best)}")

    # Guardar convergencia en CSV
    import csv
    with open("convergencia_ga.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["generacion", "fitness_max", "fitness_mean"])
        for row in convergencia:
            writer.writerow(row)

    best = tools.selBest(pop, 1)[0]
    print("\n==== Resultado final GA ====")
    print("Mejor individuo:", list(best))
    print("Mejor fitness:", best.fitness.values[0])

    best_array = np.array(list(best), dtype=float)
    np.save("mejor_individuo_fuzzy.npy", best_array)
    print("Parámetros guardados en 'mejor_individuo_fuzzy.npy'")

    return best


def demo_mejor_individuo(params_path="mejor_individuo_fuzzy.npy"):
    """Carga los mejores parámetros guardados y ejecuta una simulación en GUI.
    Útil para visualizar la marcha del controlador fuzzy optimizado.
    """

    params = np.load(params_path)
    params = params.tolist()
    print("Cargando parámetros óptimos:", params)
    set_parametros_fuzzy(params)

    # Ejecutar una simulación con GUI y guardar métricas/CSV
    metrics = run_simulation(max_steps=1500, gui=True, guardar_csv=True, csv_path="metricas_mejor_individuo.csv")
    J = funcion_objetivo(metrics)
    print(f"Función objetivo J para el mejor individuo: {J:.4f}")


if __name__ == "__main__":
    best = ga_optimize(pop_size=20, n_gen=20)
    demo_mejor_individuo()
