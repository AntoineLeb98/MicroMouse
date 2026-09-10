import numpy as np
import matplotlib.pyplot as plt
from physics.robot import Robot
from solver.controleur import Controleur_BO

## Création du robot
micromouse = Robot()
micromouse.engine_RPM = 280 #RPM
micromouse.engine_torque = 0.3923 #Nm
micromouse.mass = 100 #kg
micromouse.wheel_radius = 0.02 #m
micromouse.wheel_mass = 0.01 #kg

## Séquence des commandes
commandes = [('avance', 3), ('tourne', 'droite'), ('avance', 2)]
ctrl = Controleur_BO(micromouse, commandes)

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
axes[1,0].plot(t, state[:,2])
axes[1,0].set_title("Orientation θ")
axes[1,0].set_xlabel("Temps (s)")
axes[1,0].set_ylabel("θ (rad)")
axes[1,0].grid(True)

# Vitesse angulaire
axes[1,1].plot(t, state[:,5])
axes[1,1].set_title("Vitesse angulaire ω")
axes[1,1].set_xlabel("Temps (s)")
axes[1,1].set_ylabel("ω (rad/s)")
axes[1,1].grid(True)

fig.suptitle("États du robot dans le temps")
plt.tight_layout()
plt.show()


## Commandes moteur
