import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from physics.robot import RobotIdeal
from solver.controleur import Controleur_BO

## Création du robot
micromouse = RobotIdeal()
micromouse.engine_RPM = 280 #RPM
micromouse.engine_torque = 0.3923 #Nm
micromouse.mass = 0.5 #kg
micromouse.wheel_radius = 0.02 #m

## Création des commandes et du contrôleur
commandes = [('avance', 3), ('tourne', 'gauche'), ('avance', 5), ('tourne', 'gauche'), ('avance', 2), ('tourne', 'droite'), ('avance', 2)]
ctrl = Controleur_BO(micromouse, commandes)
ctrl.Kp_lin = 0.3
ctrl.Kd_lin = 0.2
ctrl.Kp_ang = 0.03
ctrl.Kd_ang = 0.01


## Exécution de la séquence
ctrl.execute()

## Reconstruction de state et t à partir de l'historique
state = np.array(ctrl.history)
t = np.arange(len(state)) * micromouse.dt

## Création du graphe
fig, axes = plt.subplots(2,2, figsize=(10,6))
axes_flat = axes.flatten()

# Position x et y ensemble
axes[0,0].plot(state[:,0], state[:,1])
axes[0,0].set_title("Position x-y")
axes[0,0].set_xlabel("Position x (m)")
axes[0,0].set_ylabel("Position y (m)")
axes[0,0].grid(True)

# Vitesse vx et vy ensemble
axes[0,1].plot(t, state[:,3], label="vx")
axes[0,1].plot(t, state[:,4], label="vy")
axes[0,1].set_title("Vitesse linéaire")
axes[0,1].set_xlabel("Temps (s)")
axes[0,1].set_ylabel("Vitesse (m/s)")
axes[0,1].legend()
axes[0,1].grid(True)

# Orientation
axes[1,0].plot(t, np.degrees(state[:,2]))
axes[1,0].set_title("Orientation θ")
axes[1,0].set_xlabel("Temps (s)")
axes[1,0].set_ylabel("θ (deg)")
axes[1,0].grid(True)

# Vitesse angulaire
axes[1,1].plot(t, np.degrees(state[:,5]))
axes[1,1].set_title("Vitesse angulaire ω")
axes[1,1].set_xlabel("Temps (s)")
axes[1,1].set_ylabel("ω (deg/s)")
axes[1,1].grid(True)

fig.suptitle("États du robot dans le temps")
plt.tight_layout()
plt.show()

fig, ax = plt.subplots(figsize=(6,6))
ax.plot(state[:,0], state[:,1], '--', color='gray', alpha=0.4, label="trajectoire complète")
ax.set_xlim(state[:,0].min()-0.05, state[:,0].max()+0.05)
ax.set_ylim(state[:,1].min()-0.05, state[:,1].max()+0.05)
ax.set_aspect('equal')
ax.grid(True)
ax.legend()

point, = ax.plot([], [], 'o', color='blue', markersize=8)          # position du robot
orientation, = ax.plot([], [], '-', color='red', linewidth=3)       # segment indiquant θ

L = 0.03  # longueur du segment d'orientation, ajuste selon l'échelle de ton labyrinthe

def update(frame):
    x, y, theta = state[frame,0], state[frame,1], state[frame,2]
    point.set_data([x], [y])
    orientation.set_data([x, x + L*np.cos(theta)], [y, y + L*np.sin(theta)])
    return point, orientation

pas = 1  # affiche 1 frame sur 5, pour accélérer l'animation si state est long
ani = animation.FuncAnimation(fig, update, frames=range(0, len(state), pas), interval=micromouse.dt*1000, blit=True)

plt.show()