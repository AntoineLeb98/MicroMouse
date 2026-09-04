import numpy as np

class Robot:
    def __init__(self,engine_torque,engine_RPM,mass,wheel_distance):
        self.mass = mass
        self.inertia = (mass/12)
        self.max_RPM = engine_RPM
        self.max_torque = engine_torque
        self.wheel_distance = wheel_distance
        self.state = np.empty((6,1), dtype = float)  # variables d'état = [x,y,theta,vx,vy,omega]
        self.a = 0
        self.alpha = 0 
        self.dt = 0.1 # incrémentation du temps en s
        
    def car_accels(self, F_right, F_left):
        self.a = (F_right + F_left)/self.mass
        self.alpha = (F_right - F_left)*(self.wheel_distance/2)/self.inertia

    def state_calculation(self,):
        dt = self.dt
        B = np.array([[1, 0, 0, dt, 0, 0],
                      [0, 1, 0, 0, dt, 0],
                      [0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        D = dt*np.array([[0],
                         [0],
                         [0],
                         [self.a*np.cos(self.state[2])],
                         [self.a*np.sin(self.state[2])],
                         [self.alpha]])
        self.state = B @ self.state + D #définir les nouvelles valeurs de l'état du robot

    def state_update(self, F_right, F_left):
        self.car_accels(F_right, F_left)
        self.state_calculation()


micromouse = Robot(engine_torque=0.5, engine_RPM=10000, mass=0.5, wheel_distance=0.1)
micromouse.state_update(1, 1)
print("Robot state after update:", micromouse.state)
