import numpy as np
import matplotlib.pyplot as plt

def trimf(x, a, b, c):
    return np.maximum(np.minimum((x-a)/(b-a+1e-9), (c-x)/(c-b+1e-9)), 0.0)

# --- 1) Fase de marcha (0:FL, 1:RR, 2:FR, 3:RL) ---
x_fase = np.linspace(0, 3, 400)
plt.figure(figsize=(7,4))
# Fases fuzzy: triangulares solapadas (cruce real)
plt.plot(x_fase, trimf(x_fase, 0, 0, 1.5), label='FL')
plt.plot(x_fase, trimf(x_fase, 0.5, 1, 2.0), label='RR')
plt.plot(x_fase, trimf(x_fase, 1.0, 2, 2.5), label='FR')
plt.plot(x_fase, trimf(x_fase, 1.5, 3, 3), label='RL')
plt.title('Membership functions — Gait phase')
plt.xlabel('Phase')
plt.ylabel('Membership degree')
plt.legend()
plt.grid(True, alpha=0.3); plt.ylim(-0.05,1.05); plt.tight_layout()
plt.savefig('membership_fase.png', dpi=300)

# --- 2) Error de avance (-5 a 5) ---
x_avance = np.linspace(-5, 5, 400)
plt.figure(figsize=(7,4))
# Ajuste: 'pos' reacciona antes y con más fuerza
plt.plot(x_avance, trimf(x_avance, -5, -5, -1), label='neg')
plt.plot(x_avance, trimf(x_avance, -2, 0, 2), label='cero')
plt.plot(x_avance, trimf(x_avance, 1, 5, 5), label='pos')
plt.title('Membership functions — Advance error (adjusted)')
plt.xlabel('Advance error')
plt.ylabel('Membership degree')
plt.legend()
plt.grid(True, alpha=0.3); plt.ylim(-0.05,1.05); plt.tight_layout()
plt.savefig('membership_error_avance.png', dpi=300)

# --- 3) Error de orientación (-20 a 20) ---
x_ori = np.linspace(-20, 20, 400)
plt.figure(figsize=(7,4))
plt.plot(x_ori, trimf(x_ori, -20, -20, 0), label='izq')
plt.plot(x_ori, trimf(x_ori, -10, 0, 10), label='centro')
plt.plot(x_ori, trimf(x_ori, 0, 20, 20), label='der')
plt.title('Membership functions — Orientation error')
plt.xlabel('Orientation (degrees)')
plt.ylabel('Membership degree')
plt.legend()
plt.grid(True, alpha=0.3); plt.ylim(-0.05,1.05); plt.tight_layout()
plt.savefig('membership_error_orientacion.png', dpi=300)

# --- 4) Comandos de salida (0: no, 1: sí) ---
x_cmd = np.linspace(0, 1, 200)
plt.figure(figsize=(7,4))
plt.plot(x_cmd, trimf(x_cmd, 0, 0, 1), label='no')
plt.plot(x_cmd, trimf(x_cmd, 0, 1, 1), label='si')
plt.title('Membership functions — Output commands')
plt.xlabel('Command')
plt.ylabel('Membership degree')
plt.legend()
plt.grid(True, alpha=0.3); plt.ylim(-0.05,1.05); plt.tight_layout()
plt.savefig('membership_comandos.png', dpi=300)
