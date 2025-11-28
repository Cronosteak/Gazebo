import matplotlib.pyplot as plt
import csv

# Leer datos del CSV
generaciones = []
fitness_max = []
fitness_mean = []

with open('convergencia_ga.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        generaciones.append(int(row['generacion']))
        fitness_max.append(float(row['fitness_max']))
        fitness_mean.append(float(row['fitness_mean']))

plt.figure(figsize=(8, 5))
plt.plot(generaciones, fitness_max, label='Fitness máximo', marker='o')
plt.plot(generaciones, fitness_mean, label='Fitness médio', marker='x')
plt.xlabel('Geração')
plt.ylabel('Fitness (−J)')
plt.title('Convergência do Algoritmo Genético')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('grafico_convergencia_ga.png')
plt.show()
