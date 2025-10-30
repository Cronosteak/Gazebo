import pybullet as p
import pybullet_data
import time
import os

# Inicializa PyBullet en modo GUI
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    # Carga el plano
    plane_id = p.loadURDF("plane.urdf")

    # Elige el modelo a cargar: "mini_cheetah" o "laikago"
    modelo = "laikago"  # Cambia a "mini_cheetah" si quieres tu propio modelo
    if modelo == "mini_cheetah":
        robot_path = os.path.join(os.path.dirname(__file__), "robot", "mini_cheetah.urdf")
        altura = 0.15
    else:
        robot_path = os.path.join(os.path.dirname(__file__), "robot", "laikago", "laikago.urdf")
        altura = 0.25
    robot_id = p.loadURDF(robot_path, [0, 0, altura])

    # Simula indefinidamente hasta que el usuario cierre (Ctrl+C)
    print("Simulación corriendo. Presiona Ctrl+C para cerrar.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\nSimulación finalizada por el usuario.")

    p.disconnect()

if __name__ == "__main__":
    main()
