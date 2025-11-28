# Control Fuzzy del robot Laikago – Versión 1 (pre-GA)

Este repositorio contiene la **primera versión funcional** del proyecto de control de marcha cuadrúpeda del robot Laikago usando **control difuso (fuzzy)** en PyBullet. Esta versión llega hasta la **Fase 3: Integración y ajuste de marcha**, dejando preparada la base para la futura **optimización con Algoritmos Genéticos (GA)**.

---

## 1. Objetivo de esta versión

- Implementar y probar un **controlador fuzzy** para el movimiento de avance del robot Laikago en un entorno simulado con PyBullet.
- Registrar **métricas cuantitativas** del comportamiento (trayectoria, errores, comandos) y generar **gráficos y tablas** para análisis.
- Llegar a una versión estable que sirva como **punto de partida** para la optimización de parámetros fuzzy mediante GA.

En esta etapa **NO se ha optimizado** el controlador: las reglas y funciones de pertenencia se definieron manualmente y muestran limitaciones claras (avance reducido y deriva lateral), lo cual es precisamente lo que se busca mejorar con GA.

---

## 2. Estado actual del proyecto

### 2.1. Ambiente y simulación

- **PyBullet** instalado y funcionando.
- Robot **Laikago** cargado correctamente desde el modelo URDF (`laikago/laikago_toes.urdf`).
- Simulación sobre un **plano** (`plane.urdf`) con gravedad estándar.
- Clase `LaikagoRobot` en `laikago_robot.py` que encapsula:
   - Conexión a PyBullet (GUI/DIRECT).
   - Carga del robot con una orientación inicial tal que el robot mira hacia el **eje X negativo** (giro de 180° en Z).
   - Pose inicial recomendada (hip=0, thigh≈0.67, calf≈−1.25).
   - Métodos para enviar comandos articulares (`set_joint_commands`) y leer posición y orientación de la base (`get_base_position`).
   - Un bucle de `step()` con `p.stepSimulation()`.

### 2.2. Controlador fuzzy base

Implementado en `fuzzy_controller.py` (no detallado aquí, pero considerado como caja negra en esta versión):

- Control difuso tipo **Mamdani** usando **scikit-fuzzy**.
- Controladores y salidas principales:
   - `cmd_avance`: comando global de avance (0–1) que habilita la marcha.
   - Comandos por pata para rodilla: `cmd_rodilla_FR`, `cmd_rodilla_FL`, `cmd_rodilla_RR`, `cmd_rodilla_RL`.
- Entradas principales:
   - **Fase de marcha** (0 o 1) para alternar pares diagonales de patas.
   - **Error de avance** (distancia en X respecto al punto objetivo).
- La salida `cmd_avance` se umbraliza (`> 0.3`) para decidir si el robot intenta avanzar o solo se reajusta.

### 2.3. Integración con PyBullet y lazo de control

En `simular_fuzzy_metricas.py` se implementa el **lazo cerrado** y la lógica de marcha:

- Definición de puntos:
   - `PUNTO_A = [0.0, 0.0]` (inicio).
   - `PUNTO_B = [10.0, 0.0]` (objetivo deseado sobre el eje X).
- En cada paso de simulación:
   1. Se lee la posición y orientación actual del robot (`get_base_position`).
   2. Se calcula el **error de avance** (`error_avance = PUNTO_B[0] − x_actual`).
   3. Se calcula el **ángulo objetivo** hacia `PUNTO_B` y la **orientación actual** (yaw); de ahí se deriva un **error de orientación**.
   4. Se determina la **fase de marcha** (`fase_actual`) para alternar pares diagonales de patas.
   5. Se llama al controlador fuzzy `fuzzy_step(fase_actual, error_avance)` para obtener:
       - `cmd_avance` (avance global).
       - Comandos de rodilla para cada pata.
   6. Según `cmd_avance`, se decide qué patas están en fase de avance (pares diagonales) y se aplica:
       - Limitación y sincronización de rodilla para garantizar contacto de la pata opuesta.
       - Movimiento de muslo con senoides de distinta amplitud según el estado de la rodilla.
   7. Se envían comandos articulares con `set_joint_commands`.

### 2.4. Patrón de marcha y cámara

- El patrón de marcha es **cuadrúpedo alternando pares diagonales**: FL+RR y FR+RL.
- La amplitud del muslo y límites de la rodilla están acotados para evitar posturas imposibles.
- Si `cmd_avance` es bajo, se desplaza ligeramente la base con `p.resetBasePositionAndOrientation` para evitar quedarse totalmente bloqueado.
- La cámara de PyBullet se configura para:
   - Ver al robot desde una **vista lateral horizontal**, siguiendo su posición en cada paso (`resetDebugVisualizerCamera`).

---

## 3. Registro de métricas y generación de CSV

Durante la simulación en `simular_fuzzy_metricas.py`, se registra en cada paso:

- `step`: índice de paso de simulación.
- `x`, `y`: posición de la base del robot en el plano.
- `avance`: error de avance (en X) respecto a `PUNTO_B`.
- `orientacion`: error de orientación (en grados) respecto a la dirección objetivo.
- `cmd_avance`: salida global de avance del controlador fuzzy.
- `cmd_FL`, `cmd_FR`, `cmd_RR`, `cmd_RL`: comandos fuzzy asociados a las patas.

Al finalizar (Ctrl+C o al alcanzar el objetivo), se guardan en `metricas_simulacion.csv` mediante `csv.DictWriter`.

Esto prepara el entorno para:

- **Analizar cuantitativamente** el comportamiento actual del controlador fuzzy.
- Proveer datos de entrada para la futura **optimización con GA** (función objetivo basada en estas métricas).

---

## 4. Análisis de métricas y gráficos

El análisis se realiza con `analisis_metricas.py`, que carga `metricas_simulacion.csv` y genera:

### 4.1. Gráficos generados (PNG)

1. `grafico_avance_orientacion.png`
    - Eje X: `step`.
    - Eje Y: `avance` (error de avance) y `orientacion` (error angular).
    - Observaciones actuales:
       - El **error de avance** permanece casi constante (~10), lo que indica que el robot **no se acerca de forma significativa** a `PUNTO_B`.
       - El **error de orientación** se mantiene en valores relativamente pequeños, señal de que el robot apunta aproximadamente en la dirección correcta, aunque sin avanzar mucho.

2. `grafico_comandos_fuzzy.png`
    - Eje X: `step`.
    - Eje Y: valores de `cmd_avance`, `cmd_FL`, `cmd_FR`, `cmd_RR`, `cmd_RL`.
    - Observaciones actuales:
       - `cmd_avance` es casi constante (~0.66), lo que significa que el controlador fuzzy **sigue pidiendo avance** de forma sostenida.
       - Las señales de patas (especialmente las diagonales) alternan entre dos niveles bien definidos, generando un patrón de marcha repetitivo pero que **no se traduce en mucho avance en X**.

3. `grafico_trayectoria.png`
    - Eje X: posición `x` del robot.
    - Eje Y: posición `y` del robot.
    - Observaciones actuales:
       - El cambio en `x` es muy pequeño (del orden de centímetros): el robot avanza muy poco hacia el objetivo.
       - La variación en `y` es significativamente mayor (deriva lateral), lo que confirma que la marcha actual **tiende a mover el robot hacia un lado más que hacia adelante**.

### 4.2. Métricas resumen calculadas

`analisis_metricas.py` imprime un resumen con:

- **Distancia euclidiana recorrida (m)**: distancia desde la posición inicial hasta la posición final $(x, y)$.  
   → Indica cuánto se desplazó realmente el robot en el plano.
- **Desviación lateral final (y)**: diferencia entre la `y` final e inicial.  
   → Mide cuánto se "sale" de la línea ideal de avance.
- **Avance final, medio y desviación estándar**:  
   → Miden cuánto error de avance queda al final y cuán estable fue a lo largo de la simulación.
- **Orientación final, media y desviación estándar**:  
   → Permite evaluar si el robot mantiene una orientación razonablemente estable hacia el objetivo.
- **`cmd_avance` medio**:  
   → Sirve como indicador de cuánto intenta avanzar el controlador fuzzy, independientemente del resultado físico.

En las simulaciones actuales, las métricas típicamente muestran:

- **Poca distancia euclidiana recorrida** y **desviación lateral apreciable**.
- **Error de avance alto** que apenas disminuye.
- **Comandos de avance relativamente altos**, lo que sugiere que el problema no está en la intención del controlador, sino en la **traducción de comandos a movimiento efectivo** (patrón de marcha y parámetros articulares).

---

## 5. Conclusiones de la Versión 1

1. El **pipeline completo** está implementado y funcionando:
    - Simulación en PyBullet → Control fuzzy → Comandos articulares → Registro de métricas → Gráficos y tablas.

2. El robot **genera una marcha cuadrúpeda estable**, con alternancia de patas diagonales y sin caídas catastróficas.

3. Sin embargo, el desempeño en términos de **avance hacia el objetivo** es limitado:
    - El robot se **desplaza muy poco en X**.
    - Presenta **deriva lateral** significativa (movimiento en Y).
    - El error de avance se mantiene alto, a pesar de que el controlador fuzzy ordena avanzar.

4. Esta versión cumple el rol de **baseline** (línea base) sobre la cual:
    - Se pueden definir **funciones objetivo claras** (maximizar avance útil, minimizar desviación lateral y oscilaciones).
    - Se podrán **optimizar automáticamente** las funciones de pertenencia y reglas fuzzy usando Algoritmos Genéticos.

---

## 6. Próximos pasos: Optimización GA (DEAP)

La siguiente etapa del proyecto (Fase 4 y 5 del cronograma) consistirá en:

1. **Definir la función objetivo J** basada en las métricas ya disponibles. Por ejemplo:
    - Maximizar la distancia recorrida hacia el objetivo en X.
    - Minimizar la desviación lateral (|y_final|).
    - Penalizar errores de avance/orientación altos y oscilaciones excesivas.

2. **Integrar un Algoritmo Genético** (biblioteca recomendada: `DEAP`) para optimizar:
    - Centros y anchos de las funciones de pertenencia fuzzy.
    - Posibles ganancias o pesos en las reglas.

3. **Automatizar el ciclo**:
    - GA propone un conjunto de parámetros.
    - Se ejecuta la simulación (`simular_fuzzy_metricas.py`) con esos parámetros.
    - Se calcula la función objetivo J a partir del CSV generado.
    - GA selecciona, cruza y muta soluciones hasta converger.

4. **Comparar resultados**:
    - Volver a correr `analisis_metricas.py` (o una versión extendida) para **fuzzy inicial vs fuzzy optimizado**.
    - Generar gráficos comparativos y tablas de desempeño.

Esta versión del README documenta el **punto de partida** previo a GA, incluyendo tanto la arquitectura como las limitaciones observadas, de modo que la mejora tras la optimización sea clara y cuantificable.