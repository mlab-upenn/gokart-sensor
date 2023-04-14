import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__)))
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from utils import rot_mat_2d


FAKE_GPS_NOISE = np.diag([0.1, 0.1]) ** 2 
FAKE_IMU_NOISE = np.diag([1.0, np.deg2rad(1.0)]) ** 2

class EKF:
    """
    2D EKF:
    x: [x, y, yaw, v]
    u: [v, w(yaw_rate)]
    """
    def __init__(self) -> None:
        self.R_dyn = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(0.1),  # variance of yaw angle
            2.0  # variance of velocity
        ]) ** 2  # predict state covariance
        self.Q_mea = np.diag([0.1, 0.1]) ** 2  # Observation x,y position covariance
        self.dt = 0.1

    def set_Q(self, x_var, y_var):
        self.Q_mea[0][0] = x_var
        self.Q_mea[1][1] = y_var
    
    def set_dt(self, dt):
        self.dt = dt
    
    
    def fake_input(self):
        u = np.array([[1.0], [0.1]])  # [m/s, rad/s]
        return u
    

    def fake_observation(self, xTrue, xd, u, dt):
        xTrue = self.motion_model(xTrue, u, dt)

        # add noise to gps x-y
        z = self.observation_model(xTrue) +  FAKE_GPS_NOISE @ np.random.randn(2, 1)
        # add noise to input
        ud = u + FAKE_IMU_NOISE @ np.random.randn(2, 1)

        xd = self.motion_model(xd, ud, dt)

        return xTrue, z, xd, ud

    def motion_model(self, x, u, dt):
        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[dt * math.cos(x[2, 0]), 0],
                      [dt * math.sin(x[2, 0]), 0],
                      [0.0, dt],
                      [1.0, 0.0]])

        x = F @ x + B @ u

        return x
    
    def observation_model(self, x):
        H = np.array([[1.0, 0, 0, 0],
                      [0, 1.0, 0, 0]])

        z = H @ x

        return z


    def jacob_f(self, x, u, dt):
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
            [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        return jH


    def ekf_estimation(self, xEst, PEst, z, u, dt):
        #  Predict
        xPred = self.motion_model(xEst, u, dt)
        jF = self.jacob_f(xEst, u, dt)
        PPred = jF @ PEst @ jF.T + self.R_dyn

        #  Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + self.Q_mea
        K = PPred @ jH.T @ np.linalg.inv(S)
        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
        return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    fx = rot_mat_2d(angle) @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)
    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    SIM_TIME = 50.0  # simulation time
    DT = 0.1  # time tick [s]
    ekf = EKF()
    ekf.set_dt(DT)
    
    while SIM_TIME >= time:
        time += DT
        u = ekf.fake_input()
        xTrue, z, xDR, ud = ekf.fake_observation(xTrue, xDR, u, DT)

        xEst, PEst = ekf.ekf_estimation(xEst, PEst, z, ud, DT)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))


        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(hz[0, :], hz[1, :], ".g")
        plt.plot(hxTrue[0, :].flatten(),
                    hxTrue[1, :].flatten(), "-b")
        plt.plot(hxDR[0, :].flatten(),
                    hxDR[1, :].flatten(), "-k")
        plt.plot(hxEst[0, :].flatten(),
                    hxEst[1, :].flatten(), "-r")
        plot_covariance_ellipse(xEst, PEst)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


if __name__ == '__main__':
    main()

"""
Jacobian of Motion Model
motion model
x_{t+1} = x_t+v*dt*cos(yaw)
y_{t+1} = y_t+v*dt*sin(yaw)
yaw_{t+1} = yaw_t+omega*dt
v_{t+1} = v{t}
so
dx/dyaw = -v*dt*sin(yaw)
dx/dv = dt*cos(yaw)
dy/dyaw = v*dt*cos(yaw)
dy/dv = dt*sin(yaw)
"""