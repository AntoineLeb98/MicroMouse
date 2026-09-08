import numpy as np
import matplotlib.pyplot as plt
from physics.robot import Robot

## Création des paramètres de simulation
dt = 0.1 # incrémentation du temps en s
T = 10 # durée de la simulation en s
N = int(T/dt) # nombre d'itérations de la simulation
t = np.linspace(0, T, N) # vecteur temps
state = np.zeros((N,6)) # états du robot


## Création du robot
micromouse = Robot()
micromouse.dt = dt

## Boucle de simulation
for i in range(N):
    micromouse.impulsion_moteur(0.1, 0) # application d'une impulsion moteur
    state[i,:] = micromouse.state.flatten()

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





