from physics.robot import Robot
import numpy as np

class Controleur_BO:
    def __init__(self, robot, commandes):
        self.robot = robot
        self.commandes = commandes
        self.taille_cellule = 0.3

        # Gains
        self.Kp_lin = 0.1
        self.Kd_lin = 0
        self.Kp_ang = 0.1
        self.Kd_ang = 0

        # Tolérances de convergence
        self.tol_pos = 0.1
        self.tol_vit = 0.1
        self.tol_angle = 0.1
        self.tol_omega = 0.1

        # Commandes
        self.history = []

    def avance(self, n_cellules):
        consigne_distance = n_cellules * self.taille_cellule
        position_initiale = self.robot_position()
        compteur = 0
        max_iter = 5000

        while True:
            distance_parcourue = np.linalg.norm(self.robot_position() - position_initiale)
            erreur = consigne_distance - distance_parcourue
            v = self.robot.vehicule_fwd_spd()

            if abs(erreur) < self.tol_pos and abs(v) < self.tol_vit:
                break

            duty = np.clip(self.Kp_lin * erreur - self.Kd_lin * v, -1, 1)
            self._step(duty,duty)

            compteur += 1
            if compteur >= max_iter:
                print(f"avance({n_cellules}) n'a pas convergé après {compteur} itérations — erreur={erreur:.4f}, v={v:.4f}")
                break

        self._step(0,0)

    def tourne(self, direction):
        theta_mod = {'gauche': np.pi/2, 'droite':-np.pi/2, 'demi-tour': np.pi}[direction]
        theta_consigne = self.robot_theta() + theta_mod
        compteur = 0
        max_iter = 5000

        while True:
            erreur = theta_consigne - self.robot_theta()
            omega = self.robot_omega()

            if abs(erreur) < self.tol_angle and abs(omega) < self.tol_omega:
                break

            duty = np.clip(self.Kp_ang * erreur - self.Kd_ang * omega, -1, 1)
            self._step(-duty,duty)

            compteur += 1

            if compteur >= max_iter:
                print(f"tourne({direction}) n'a pas convergé après {compteur} itérations — erreur={erreur:.4f}, omega={omega:.4f}")
                break

        self._step(0,0)

    def _step(self, duty_gauche, duty_droite):
        self.robot.impulsion_moteur(duty_gauche, duty_droite)
        self.history.append(self.robot.state.flatten().copy())

    def execute(self):
        self.history.append(self.robot.state.flatten().copy())  # état de départ
        for action, valeur in self.commandes:
            if action == 'avance':
                self.avance(valeur)
            elif action == 'tourne':
                self.tourne(valeur)

    def robot_position(self):
        position_actuelle = np.array([self.robot.state[0,0], self.robot.state[1,0]])
        return position_actuelle

    def robot_vitesse(self):
        vitesse_actuelle = np.array([self.robot.state[3,0], self.robot.state[4,0]])
        return vitesse_actuelle

    def robot_theta(self):
        position_angulaire = self.robot.state[2,0]
        return position_angulaire

    def robot_omega(self):
        vitesse_angulaire = self.robot.state[5,0]
        return vitesse_angulaire