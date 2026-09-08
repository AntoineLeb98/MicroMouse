import numpy as np

class Robot:
    def __init__(self):
        ## Paramètres robots
        self.mass = 0.5 #masse du robot en kg
        self.robot_length = 0.1 #longueur du robot
        self.robot_width = 0.1 #largeur du robot au niveau des roues
        self.inertia = (self.mass/12)*(self.robot_length**2 + self.robot.width**2) #inertie du robot en kg.m^2

        ## Paramètres moteurs/roue
        self.engine_RPM = 10000 #RPM du moteur
        self.omega_max = self.engine_RPM * (2*np.pi/60)  # conversion RPM → rad/s
        self.engine_torque = 0.1 #Nm du moteur
        self.wheel_mass = 0.01 #masse de la roue en kg
        self.wheel_radius = 0.1 #rayon des roues
        self.I_wheel = 0.5*self.mass*self.wheel_radius**2 

        ## Calcul des états et de l'accélération
        self.state = np.zeros((6,1), dtype = float)  # variables d'état = [x,y,theta,vx,vy,omega]
        self.wheel_speeds = np.zeros((2,1)) # [omega_gauche, omega_droite]
        self.a = 0 #acceleration linéaire du robot en m/s^2
        self.alpha = 0 #acceleration angulaire du robot en rad/s^2
        self.dt = 0.1 # incrémentation du temps en s

        ## Gains contrôleurs
        self.Kp_wheel = 1 #

        ## Constantes physiques
        self.g = 9.81 #constante gravitationnelle en m/s**2
        self.mu = 0.8 #constante de friction

    def motor_output(self, duty_cmd):
        duty_cmd = np.clip(duty_cmd, -1, 1)
        t_available = self.engine_torque * (duty_cmd - self.wheel_speeds/self.omega_max)
        



    def robot_accels(self, F_right, F_left):
        self.a = (F_right + F_left)/self.mass
        self.alpha = (F_right - F_left)*(self.robot_width/2)/self.inertia

    def state_calculation(self):
        B = np.array([[1, 0, 0, dt, 0, 0],
                      [0, 1, 0, 0, dt, 0],
                      [0, 0, 1, 0, 0, dt],
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

    def impulsion_moteur(self, F_right, F_left):
        self.robot_accels(F_right, F_left)
        self.state_calculation()