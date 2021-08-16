import abc
import numpy as np
from collections import deque
import scipy.stats


class FilterBase(abc.ABC):
    def __init__(self, **kwargs):
        pass
    
    def update(self, z):
        raise NotImplementedError

    def get(self):
        raise NotImplementedError


class AverageFilter(FilterBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.k = 1
        self.avg = 0

    def update(self, x):
        if self.k == 1:
            self.avg = x
        else:
            alpha = (self.k - 1) / self.k
            self.avg = alpha * self.avg + (1 - alpha) * x
        self.k += 1
        return self.avg

    def get(self):
        return self.avg


class MovingAverageFilter(FilterBase):
    def __init__(self, n=10, **kwargs):
        super().__init__(**kwargs)
        self.n = n
        self.clear()
        
    def clear(self):
        self.buf = None
        self.avg = 0
    
    def update(self, x):
        if self.buf is None:
            self.buf = deque([x] * self.n)
            self.avg = x
        else:
            self.buf.append(x)
            self.avg += (x - self.buf.popleft()) / self.n

    def get(self):
        return self.avg


class MovingAverageFilter2(FilterBase):
    def __init__(self, n=10, **kwargs):
        super().__init__(**kwargs)
        self.n = n
        self.clear()
        
    def clear(self):
        self.buf = None
        self.avg = 0
    
    def update(self, x):
        if self.buf is None:
            self.buf = deque([x] * self.n)
            self.avg = x
        else:
            self.buf.append(x)
            self.buf.popleft()
            self.avg = sum(self.buf) / self.n

    def get(self):
        return self.avg


class LowPassFilter(FilterBase):
    def __init__(self, alpha=0.7, **kwargs):
        super().__init__(**kwargs)
        self.alpha = alpha
        self.clear()
    
    def clear(self):
        self.empty = True
        self.z = 0
        
    def update(self, z):
        if self.empty:
            self.z = z
            self.empty = False
        else:
            self.z = self.alpha * self.z + (1 - self.alpha) * z

    def get(self):
        return self.z



class HighPassFilter(FilterBase):
    def __init__(self, tau=0.0223, dt=0.01, **kwargs):
        super().__init__(**kwargs)
        self.tau = tau
        self.dt = 0.01
        self.clear()

    def clear(self):
        self.empty = True
        self.prev_x = 0
        self.prev_u = 0
        
    def update(self, u):
        alpha = self.tau / (self.tau + self.dt)
        self.prev_x = alpha * self.prev_x + alpha * (u - self.prev_u)
        self.prev_u = u

    def get(self):
        return self.prev_x


class SimpleKalman(FilterBase):
    def __init__(self, x0=14, P0=6, A=1, H=1, Q=0, R=4, **kwargs):
        super().__init__(**kwargs)
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        
        self.x = x0
        self.P = P0
        self.K = None
        
    def update(self, z):
        xp = self.A * self.x
        Pp = self.A * self.P * self.A + self.Q
        
        K = Pp * self.H / (self.H * Pp * self.H + self.R)
        
        x = xp + K * (z - self.H*xp)
        P = Pp - K * self.H * Pp
        
        self.x = x
        self.P = P
        self.K = K

    def get(self):
        return self.x


class KalmanFilter(FilterBase):
    def __init__(self, x0, P0, A, H, Q, R, **kwargs):
        super().__init__(**kwargs)
        self.x = x0
        self.P = P0
        
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        
    def update(self, z):
        A, H, Q, R = self.A, self.H, self.Q, self.R
        x, P = self.x, self.P
        
        x_pred = A @ x
        P_pred = A @ P @ A.T + Q
        
        K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
        
        self.x = x_pred + K @ (z - H @ x_pred)
        self.P = P_pred - K @ H @ P_pred

    def get(self):
        return self.H @ self.x


class ExtendedKalmanFilter(FilterBase):
    def __init__(self, x0, P0, Q, R, A=None, H=None, **kwargs):
        super().__init__(**kwargs)
        self.x = x0
        self.P = P0

        self.A_ = A
        self.H_ = H
        self.Q_ = Q
        self.R_ = R

    def update(self, z):
        A, H, Q, R = self.A, self.H, self.Q_, self.R_
        x, P = self.x, self.P

        x_pred = self.f(x)
        P_pred = A @ P @ A.T + Q
        
        K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)

        self.x = x_pred + K @ (z - self.h(x_pred))
        self.P = P_pred - K @ H @ P_pred

    @property
    def A(self):
        if self.A_ is not None:
            return self.A_
        raise NotImplementedError
        
    @property
    def H(self):
        if self.H_ is not None:
            return self.H_
        raise NotImplementedError
        
    def f(self, x):
        if self.A_ is not None:
            return self.A_ @ x
        raise NotImplementedError
    
    def h(self, x):
        if self.H_ is not None:
            return self.H_ @ x
        raise NotImplementedError

    def get(self):
        return self.h(self.x)


class UnscentedKalmanFilter(FilterBase):
    def __init__(self, x0, P0, Q, R, **kwargs):
        super().__init__(**kwargs)
        self.x = x0
        self.P = P0

        self.Q_ = Q
        self.R_ = R
    
    @staticmethod
    def sigma_points(x, P, kappa):
        n = x.size
        U = np.linalg.cholesky((n + kappa) * P).T  # https://stackoverflow.com/a/16699500
        
        x = x.reshape(n)
        chi = [x]
        W = [kappa / (n+kappa)]
        for k in range(n):
            chi.append(x + U[k, :])
            W.append(1 / (2 * (n+kappa)))
        for k in range(n):
            chi.append(x - U[k, :])
            W.append(1 / (2 * (n+kappa)))
        
        chi = np.stack(chi, axis=1)
        W = np.array(W)
        return chi, W
    
    @staticmethod
    def UT(chi, W, noise_cov=0):
        xm = (W * chi).sum(axis=1).reshape(-1, 1)
        mat = chi - xm
        xcov = np.einsum('k,ik,kj->ij', W, mat, mat.T)
        
        return xm, xcov + noise_cov

    def update(self, z):
        Q, R = self.Q_, self.R_
        
        chi, W = UnscentedKalmanFilter.sigma_points(self.x, self.P, 0)
        f_chi = self.f(chi)
        x_pred, P_pred = UnscentedKalmanFilter.UT(f_chi, W, Q)
        
        h_chi = self.h(chi)
        z_pred, Pz = UnscentedKalmanFilter.UT(h_chi, W, R)
        
        Pxz = np.einsum('k,ik,jk->ij', W, f_chi - x_pred, h_chi - z_pred)
    
        K = Pxz @ np.linalg.inv(Pz)

        self.x = x_pred + K @ (z - z_pred)
        self.P = P_pred - K @ Pz @ K.T

    def f(self, x):
        raise NotImplementedError
    
    def h(self, x):
        raise NotImplementedError

    def get(self):
        return self.h(self.x)


class ParticleFilter(FilterBase):
    def __init__(self, x0, n_pts=1000, sampling_method='sir', **kwargs):
        super().__init__(**kwargs)
        self.x = x0
        self.sampling_method = sampling_method
        
        n = x0.shape[0]
        self.pts = x0 + 0.1 * x0 * np.random.normal(0, 1, (x0.shape[0], n_pts))
        self.wt = np.ones((1, n_pts)) / n_pts
        
    def update(self, z):
        pts = self.f(self.pts) + np.random.normal(0, 1, self.pts.shape)
        rv = scipy.stats.multivariate_normal(z.reshape(-1), 100)
        wt = self.wt * rv.pdf(self.h(pts).T)[None]
        if (wt == 0).all():
            wt = np.ones((1, pts.shape[1])) / pts.shape[1]
        else:
            wt /= wt.sum()
        self.x = pts @ wt.T
        self.pts, self.wt = self.resample(pts, wt)
        
    def resample(self, pts, wt):
        if self.sampling_method == 'sir':
            return ParticleFilter.sequential_important_resampling(pts, wt)
        raise NotImplementedError
    
    @staticmethod
    def sequential_important_resampling(pts, wt):
        n_pts = wt.size
        idxs = np.random.choice(n_pts, n_pts, replace=True, p=wt.reshape(-1))
        resampled_pts = pts[:, idxs]
        wt = np.ones((1, n_pts)) / n_pts
        return resampled_pts, wt
    
    def f(self, x):
        raise NotImplementedError
    
    def h(self, x):
        raise NotImplementedError

    def get(self):
        return self.h(self.x)
