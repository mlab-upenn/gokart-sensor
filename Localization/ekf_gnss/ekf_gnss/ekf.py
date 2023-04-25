import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__)))
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
# from utils import rot_mat_2d
from scipy.spatial.transform import Rotation as Rot

FAKE_GPS_NOISE = np.diag([0.1, 0.1, 0.1,0.1,0.1]) ** 2 
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
            1.0,  # variance of x velocity
            0.1  # variance of yaw angular velocity
        ]) ** 2  # predict state covariance
        self.Q_mea = np.diag([0.1, 0.1, 1e-5, 0.1, 1e-5]) ** 2  # Observation x,y position covariance
        self.dt = 0.1   
    
    def set_Q(self, x_var, y_var):
        # we know that imu variance is not changing and only GPS covar is changing
        # so only setting GPS variance as it fluctuates 
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
        z = self.observation_model(xTrue) +  FAKE_GPS_NOISE @ np.random.randn(5, 1)
        # add noise to input
        ud = u + FAKE_IMU_NOISE @ np.random.randn(2, 1)
        # print(z)

        xd = self.motion_model(xd, ud, dt)

        return xTrue, z, xd, ud

    def motion_model(self, x, u, dt):
        F = np.array([[1.0, 0, 0, 0, 0],
                    [0, 1.0, 0, 0,0],
                    [0, 0, 1.0, 0,0],
                    [0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0]])

        B = np.array([[dt * math.cos(x[2, 0]), 0],
                      [dt * math.sin(x[2, 0]), 0],
                      [0.0, dt],
                      [1.0, 0.0],
                      [0.0,1.0]])

        x = F @ x + B @ u

        return x
    
    def observation_model(self, x):
        H = np.array([[1.0, 0, 0, 0,0],
                      [0, 1.0, 0, 0,0],
                      [0, 0, 1.0, 0,0],
                      [0, 0, 0, 1.0,0],
                      [0, 0, 0, 0,1.0]])

        z = H @ x

        return z


    def jacob_f(self, x, u, dt):
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw),0],
            [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw),0],
            [0.0, 0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 0.0, 1.0,0.0],
            [0.0, 0.0, 0.0, 0.0,1.0]])

        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0,0],
            [0, 1, 0, 0,0],
            [0, 0, 1, 0,0],
            [0, 0, 0, 1,0],
            [0, 0, 0, 0,1]])

        return jH
    
    def ekf_predict(self, xEst, PEst, u, dt):
        #  Predict
        xPred = self.motion_model(xEst, u, dt)
        jF = self.jacob_f(xEst, u, dt)
        PPred = jF @ PEst @ jF.T + self.R_dyn
        return xPred, PPred
    
    def ekf_partial_update(self, xEst, PEst, z_partial, z_bool, m_covar):
        #  Update
        assert z_bool.shape[0] ==5 #need to make sure we know what elements are to be updates
        num_param = np.zeros((5,1))[z_bool,:].shape[0] #find number of True's in vec 
        assert num_param>0 #atlease one update
        assert m_covar.shape[0] == num_param #if 2 elements are to be updates, covar should be 2*2
        assert m_covar.shape[1] == num_param
        x_par = xEst[z_bool,:]
        #the observation model is direct, so it will be unity
        #here we are observing partial states directly, so jacobian will be unity
        zPred = x_par
        jH = np.eye(5)[z_bool,:]
        S = jH @ PEst @ jH.T + m_covar
        S_inv = np.linalg.inv(S)
        K = PEst @ jH.T @ S_inv

        y = z_partial - zPred #what partially we observed minus what partially was predicted
        xEst = xEst + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PEst
        return xEst, PEst       


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
        S_inv = np.linalg.inv(S)

        K = PPred @ jH.T @ S_inv
        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
        return xEst, PEst


def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2] 

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

    # State Vector [x y yaw v omega]'
    xEst = np.zeros((5, 1))
    xTrue = np.zeros((5, 1))
    PEst = np.eye(5)
    xDR = np.zeros((5, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((5, 1))

    SIM_TIME = 50.0  # simulation time
    DT = 0.1  # time tick [s]
    ekf = EKF()
    ekf.set_dt(DT)
    import sys
    
    while SIM_TIME >= time:
        time += DT
        u = ekf.fake_input()
        xTrue, z, xDR, ud = ekf.fake_observation(xTrue, xDR, u, DT)

        #test for partial update
        # z_p = z[2:4]
        # z_covar = np.eye(2)*0.1
        # z_b = np.array([False, False, True, True, False])
        # print("before")
        # print("xEst")
        # print(xEst)
        # print("PEst")
        # print(PEst)
        
        # xEst, PEst = ekf.ekf_partial_update(xEst, PEst, z_p, z_b, z_covar)

        # print("after")
        # print("xEst")
        # print(xEst)
        # print("PEst")
        # print(PEst)
        # sys.exit()
        #test for partial update ends


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