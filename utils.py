import scipy.io
from matplotlib import pyplot as plt
import numpy as np


def load_imu_data():
    dt = 0.01
    gyro_data = scipy.io.loadmat('./source/11.ARS/ArsGyro.mat')
    acce_data = scipy.io.loadmat('./source/11.ARS/ArsAccel.mat')

    ts = np.arange(len(gyro_data['wz'])) * dt
    gyro = np.concatenate([
        gyro_data['wx'],
        gyro_data['wy'],
        gyro_data['wz'],
    ], axis=1)
    acce = np.concatenate([
        acce_data['fx'],
        acce_data['fy'],
        acce_data['fz'],
    ], axis=1)

    return dt, ts, gyro, acce


def load_sonar_data():
    sonar_data = scipy.io.loadmat('./source/2.MovAvgFilter/SonarAlt.mat')['sonarAlt'].reshape(-1)[:500]
    dt = 0.02
    ts = np.arange(len(sonar_data)) * dt
    return dt, ts, sonar_data[:500]


def generate_volt_data():
    while True:
        yield np.random.normal(14.4, 4)
        

def generate_pos_vel_data(dt=0.1):
    pos = 0
    vel = 80
    while True:
        w = np.random.normal(0, 10)
        v = np.random.normal(0, 10)
        pos += vel * dt
        yield pos + v, vel
        vel = 80 + w


def generate_radar_measurement_data(dt):
    pos = 0
    while True:
        vel = np.random.normal(100, 5)
        alt = np.random.normal(1000, 10)
        pos = pos + vel*dt
        v = np.random.normal(0, pos * 0.05)
        r = (pos**2 + alt**2)**0.5 + v
        yield r


def run_radar_position_estimation(kf, ts, measurements_seq):
    measurements = []
    estimations = []

    speeds = []
    altitudes = []
    positions = []

    for t, meas in zip(ts, measurements_seq):
        kf.update(np.array([[meas]]))
        
        state = kf.x.copy()
        
        measurements.append(meas)
        estimations.append(kf.h(state)[0, 0])
        
        pos, spd, alt = state.reshape(3)
        positions.append(pos)
        speeds.append(spd)
        altitudes.append(alt)

    return measurements, estimations, speeds, altitudes, positions


def run_euler_attitude_estimation(kf, ts, gyro, acce):
    estimations = []
    for i, (g, a) in enumerate(zip(gyro, euler_from_acce(acce))):
        kf.gyro = g.reshape(3, 1)
        kf.update(a[:2].reshape(2, 1))
        estimations.append(kf.get().reshape(1, 2))
    return np.concatenate(estimations) * 180 / np.pi


def plot_xyz(ts, xyz, title=''):
    fig = plt.figure(figsize=[16, 12])
    fig.suptitle(title)
    for i, ax, color in zip(range(xyz.shape[1]), 'xyz', 'rgb'):
        fig.add_subplot(3, 1, i+1)
        plt.plot(ts, xyz[:, i], color=color)
        plt.ylabel(ax)
        plt.xlabel('Time[sec]')
    plt.show()


def plot_radar_result(ts, speeds, altitudes, positions):
    def plot(ts, values, ylabel):
        plt.figure(figsize=[12, 6])
        plt.plot(ts, values)
        plt.xlabel('Time[sec]')
        plt.ylabel(ylabel)
        plt.show()
    
    plot(ts, speeds, 'Speed[m/s]')
    plot(ts, altitudes, 'Altitude[m]')
    plot(ts, positions, 'Position[m]')


def plot_measurement_vs_estimation(ts, measurements, estimations, ylabel=''):
    plt.figure(figsize=[12, 9])
    plt.plot(ts, measurements, '--', label='measured')
    plt.plot(ts, estimations, label='estimated')
    plt.xlabel('Time[sec]')
    plt.ylabel(ylabel)
    plt.legend()
    plt.show()


def euler_from_gyro(ts, gyro):
    attitude = np.array([[0, 0, 0]]).T
    res = np.zeros((len(ts), 3), dtype=float)
    
    for i, (dt, pqr) in enumerate(zip(ts[1:] - ts[:-1], gyro)):
        phi, theta, _ = attitude.reshape(-1)
        
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        cos_theta = np.cos(theta)
        tan_theta = np.tan(theta)
        
        to_euler = np.array([
            [1, sin_phi * tan_theta, cos_phi * tan_theta],
            [0, cos_phi, -sin_phi],
            [0, sin_phi * cos_theta, cos_phi * cos_theta],
        ])
        attitude = attitude + dt * to_euler @ pqr.reshape(3, 1)
        res[i+1] = attitude.reshape(-1)
        
    return res


def euler_from_acce(acce):
    g = 9.8
    theta = np.arcsin(acce[:, 0] / g)
    phi = np.arcsin(-acce[:, 1] / (g * np.cos(theta)))
    return np.stack([phi, theta, np.zeros_like(phi)], axis=1)


def euler_from_acce2(acce):
    x, y, z = acce.T
    phi = np.arctan2(y, z)
    theta = np.arctan2(x, (y**2 + z**2)**0.5)
    return np.stack([phi, theta, np.zeros_like(phi)], axis=1)
