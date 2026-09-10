import numpy as np

class Robot:
    def __init__(self):
        ## Paramètres robots
        self.mass = 0.5 #masse du robot en kg
        self.robot_length = 0.1 #longueur du robot
        self.robot_width = 0.1 #largeur du robot au niveau des roues
        self.inertia = (self.mass/12)*(self.robot_length**2 + self.robot_width**2) #inertie du robot en kg.m^2

        ## Paramètres moteurs/roue
        self.engine_RPM = 10000 #RPM du moteur
        self.omega_max = self.engine_RPM * (2*np.pi/60)  # conversion RPM → rad/s
        self.engine_torque = 0.1 #Nm du moteur
        self.wheel_mass = 0.01 #masse de la roue en kg
        self.wheel_radius = 0.02 #rayon des roues
        self.I_wheel = 0.5*self.wheel_mass*self.wheel_radius**2 

        ## Calcul des états et de l'accélération
        self.state = np.zeros((6,1), dtype = float)  # variables d'état = [x,y,theta,vx,vy,omega]
        self.wheel_speeds = np.zeros((2,1)) # [omega_gauche, omega_droite]
        self.a = 0 #acceleration linéaire du robot en m/s^2
        self.alpha = 0 #acceleration angulaire du robot en rad/s^2
        self.dt = 0.1 # incrémentation du temps en s

        ## Constantes physiques
        self.g = 9.81 #constante gravitationnelle en m/s**2
        self.mu = 0.8 #constante de friction

    def vehicule_fwd_spd(self):
        speed_vector = np.array([self.state[3,0],self.state[4,0]])
        unit_direction = np.array([np.cos(self.state[2,0]),np.sin(self.state[2,0])])
        vfs = np.dot(speed_vector, unit_direction)
        return vfs

    def wheel_dynamics(self, duty_gauche, duty_droite):
        duty_cmd = np.clip(np.array([[duty_gauche],[duty_droite]]), -1, 1)

        # Vitesse du châssis
        v_avant = self.vehicule_fwd_spd()

        # Vitesse des roues au point de contact sans frottement
        offset = np.array([[self.robot_width/2],[-self.robot_width/2]])
        v_contact = v_avant - self.state[5,0]*offset

        # Vitesse des roues mesurée
        v_surface = self.wheel_speeds * self.wheel_radius

        # Mesure du glissement
        glissement = v_surface - v_contact

        # Force de friction (Coulomb simple, pour l'instant)
        N = (self.mass * self.g) / 2
        F = self.mu * N * np.sign(glissement)

        # Couple moteur
        torque = self.motor_output(duty_cmd)

        # Bilan sur la roue -> intégration de wheel_speeds
        T_net = torque - F*self.wheel_radius
        alpha_roue = T_net / self.I_wheel
        self.wheel_speeds = self.wheel_speeds + alpha_roue*self.dt

        return F


    def motor_output(self, duty_cmd):
        t_available = self.engine_torque * (duty_cmd - self.wheel_speeds/self.omega_max)
        return t_available

    def robot_accels(self, F_right, F_left):
        self.a = (F_right + F_left)/self.mass
        self.alpha = (F_right - F_left)*(self.robot_width/2)/self.inertia

    def impulsion_moteur(self, duty_gauche, duty_droite):
        F = self.wheel_dynamics(duty_gauche, duty_droite)
        self.robot_accels(F[1,0], F[0,0])  # F[0,0]=gauche, F[1,0]=droite selon ta convention offset
        self.state_calculation()

    def state_calculation(self):
        B = np.array([[1, 0, 0, self.dt, 0, 0],
                      [0, 1, 0, 0, self.dt, 0],
                      [0, 0, 1, 0, 0, self.dt],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        D = self.dt*np.array([[0],
                         [0],
                         [0],
                         [self.a*np.cos(self.state[2,0])],
                         [self.a*np.sin(self.state[2,0])],
                         [self.alpha]])
        self.state = B @ self.state + D #définir les nouvelles valeurs de l'état du robot

    def print_state(self):
        print("État du robot : ") 
        print("Position en x : ", self.state[0,0])
        print("Position en y : ", self.state[1,0])
        print("Orientation : ", self.state[2,0])
        print("Vitesse en x : ", self.state[3,0])
        print("Vitesse en y : ", self.state[4,0])
        print("Vitesse angulaire : ", self.state[5,0]) 

    