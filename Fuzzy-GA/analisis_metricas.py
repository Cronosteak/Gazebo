import pandas as pd
import matplotlib.pyplot as plt

# Cargar métricas desde CSV
metricas = pd.read_csv('metricas_simulacion.csv')


# Calcular distancia euclidiana recorrida (inicio a fin)
def calcular_distancia_euclidiana(metricas):
    x0, y0 = metricas['x'].iloc[0], metricas['y'].iloc[0]
    xf, yf = metricas['x'].iloc[-1], metricas['y'].iloc[-1]
    return ((xf - x0)**2 + (yf - y0)**2)**0.5

# Calcular desviación lateral (en Y)
def calcular_desviacion_lateral(metricas):
    y0 = metricas['y'].iloc[0]
    yf = metricas['y'].iloc[-1]
    return yf - y0

# Graficar trayectoria X vs Y
def plot_trayectoria(metricas):
    plt.figure(figsize=(8,4))
    plt.plot(metricas['x'], metricas['y'], marker='o', markersize=2, label='Trayectoria')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trayectoria del robot (X vs Y)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('grafico_trayectoria.png')
    plt.close()

import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def cargar_metricas(csv_path='metricas_simulacion.csv'):
    df = pd.read_csv(csv_path)
    return df


def resumen_metricas(df):
    x0, y0 = df['x'].iloc[0], df['y'].iloc[0]
    xf, yf = df['x'].iloc[-1], df['y'].iloc[-1]
    dist = np.sqrt((xf - x0) ** 2 + (yf - y0) ** 2)
    desvio_lateral = yf - y0
    avance_final = df['avance'].iloc[-1]
    avance_mean = df['avance'].mean()
    ori_final = df['orientacion'].iloc[-1]
    ori_mean = df['orientacion'].abs().mean()

    resumen = {
        'x_inicial': x0,
        'y_inicial': y0,
        'x_final': xf,
        'y_final': yf,
        'distancia_recorrida_xy': dist,
        'desvio_lateral_final': desvio_lateral,
        'avance_final': avance_final,
        'avance_medio': avance_mean,
        'orientacion_final': ori_final,
        'orientacion_media_abs': ori_mean,
        'cmd_avance_medio': df['cmd_avance'].mean(),
    }
    return resumen


def graficar_avance_orientacion(df, output_path='grafico_avance_orientacion.png'):
    plt.figure(figsize=(10, 5))
    plt.plot(df['step'], df['avance'], label='Error de avance (X objetivo - X)')
    plt.plot(df['step'], df['orientacion'], label='Error de orientación (deg)')
    plt.xlabel('Paso')
    plt.ylabel('Valor')
    plt.title('Evolución del error de avance y orientación')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def graficar_comandos(df, output_path='grafico_comandos_fuzzy.png'):
    plt.figure(figsize=(10, 5))
    plt.plot(df['step'], df['cmd_avance'], label='cmd_avance')
    plt.plot(df['step'], df['cmd_FL'], label='cmd_FL')
    plt.plot(df['step'], df['cmd_FR'], label='cmd_FR')
    plt.plot(df['step'], df['cmd_RR'], label='cmd_RR')
    plt.plot(df['step'], df['cmd_RL'], label='cmd_RL')
    plt.xlabel('Paso')
    plt.ylabel('Comando difuso')
    plt.title('Evolución de los comandos difusos')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def graficar_trayectoria(df, output_path='grafico_trayectoria.png'):
    plt.figure(figsize=(6, 6))
    plt.plot(df['x'], df['y'], marker='o', markersize=2, label='Trayectoria')
    plt.scatter([df['x'].iloc[0]], [df['y'].iloc[0]], color='green', label='Inicio')
    plt.scatter([df['x'].iloc[-1]], [df['y'].iloc[-1]], color='red', label='Final')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trayectoria en el plano XY')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def comparar_resumenes(resumen_base, resumen_opt):
    """Imprime un resumen comparativo base vs optimizado."""

    print("\n==== Comparación controlador fuzzy original vs mejor individuo GA ====")
    claves = [
        'distancia_recorrida_xy',
        'desvio_lateral_final',
        'avance_final',
        'avance_medio',
        'orientacion_final',
        'orientacion_media_abs',
        'cmd_avance_medio',
    ]

    for k in claves:
        vb = resumen_base.get(k, None)
        vo = resumen_opt.get(k, None)
        print(f"{k}:")
        print(f"  base      = {vb}")
        print(f"  optimizado= {vo}")


def graficar_trayectorias_comparadas(df_base, df_opt, output_path='grafico_trayectorias_comparadas.png'):
    plt.figure(figsize=(6, 6))
    plt.plot(df_base['x'], df_base['y'], label='Base', alpha=0.7)
    plt.plot(df_opt['x'], df_opt['y'], label='Optimizado', alpha=0.7)
    plt.scatter([df_base['x'].iloc[0]], [df_base['y'].iloc[0]], color='green', label='Inicio')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trayectorias comparadas (base vs optimizado)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


if __name__ == '__main__':
    # Cargar métricas del controlador fuzzy original
    df_base = cargar_metricas('metricas_simulacion.csv')
    resumen_base = resumen_metricas(df_base)

    print("Resumen de métricas de la simulación base (controlador fuzzy original):")
    for k, v in resumen_base.items():
        print(f"  {k}: {v}")

    graficar_avance_orientacion(df_base, 'grafico_avance_orientacion_base.png')
    graficar_comandos(df_base, 'grafico_comandos_fuzzy_base.png')
    graficar_trayectoria(df_base, 'grafico_trayectoria_base.png')

    print("Gráficos base guardados: 'grafico_avance_orientacion_base.png', 'grafico_comandos_fuzzy_base.png', 'grafico_trayectoria_base.png'")

    # Si existen métricas del mejor individuo, cargarlas y comparar
    if os.path.exists('metricas_mejor_individuo.csv'):
        df_opt = cargar_metricas('metricas_mejor_individuo.csv')
        resumen_opt = resumen_metricas(df_opt)

        print("\nResumen de métricas de la simulación optimizada (mejor individuo GA):")
        for k, v in resumen_opt.items():
            print(f"  {k}: {v}")

        comparar_resumenes(resumen_base, resumen_opt)
        graficar_trayectorias_comparadas(df_base, df_opt, 'grafico_trayectorias_comparadas.png')
        print("Gráfico comparativo de trayectorias guardado en 'grafico_trayectorias_comparadas.png'")
    else:
        print("\nNo se encontró 'metricas_mejor_individuo.csv'. Ejecuta primero la demo del mejor individuo para generarlo.")
