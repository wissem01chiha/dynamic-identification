import numpy as np
import matplotlib.pyplot as plt 

class Trapezoidal:
    """
    Base class for trapezoidal trajectories generation 
    Args:
        - njoints     number of joints
        - nwaypoints  number of waypoints
        
                    acceleration values
                    accelerated durations
                    vel constant durations
                    runtime
    Output:
                    q, qd, qdd
    """

    def __init__(self, njoints, nwaypoints, acc, delta_t1, delta_t2, Nf):
        self.njoints = njoints

        self.q0 = np.random.uniform(-np.pi / 2, np.pi / 2, size=(njoints,))
        self.qd0 = np.zeros(njoints)
        self.qdd0 = acc[0, :]

        self.Kq = []
        self.Kv = []
        self.Ka = []
        # (nwaypoints -1 x njoints), matrix of acceleratiion on 1st accelearated
        # duration
        self.acc = acc
        # (nwaypoints -1 x njoints)1st accelerated duration
        self.delta_t1 = delta_t1
        # (nwaypoints -1 x njoints)constant vel duration
        self.delta_t2 = delta_t2

        self.nwaypoints = nwaypoints
        # (nwaypoints - 1x njoints)a list of runtime between 2 consecutive waypoints
        self.Nf = Nf
        self.ts = 0.01

    def Trapezoidal(self):
        self.initConfig()
        if self.nwaypoints == 1:
            print("Number of waypoints needs to be more 1!")
        else:
            for i in range(self.nwaypoints - 1):
                for j in range(self.njoints):
                    # at one joint between 2 waypoints
                    q_, qd_, qdd_ = self.trapTraj_PTP(
                        self.acc[i, j],
                        self.q[j][-1],
                        self.delta_t1[i, j],
                        self.delta_t2[i, j],
                        self.Nf[i],
                    )
                    self.q[j] = np.append(self.q[j], q_)
                    self.qd[j] = np.append(self.qd[j], qd_)
                    self.qdd[j] = np.append(self.qdd[j], qdd_)
        # self.plotTraj()
        return self.q, self.qd, self.qdd

    def initConfig(self):
        self.q = []
        self.qd = []
        self.qdd = []
        for i in range(self.njoints):
            self.q.append(np.array([self.q0[i]]))
            self.qd.append(np.array([self.qd0[i]]))
            self.qdd.append(np.array([self.qdd0[i]]))

    def trapTraj_PTP(self, a1, q0, n1, n2, N):
        ts = self.ts
        q_ = np.array([q0])
        qd_ = np.array([0])
        qdd_ = np.array([a1])
        # acceleration on 2nd accelarated duration to ensure vel(end) = 0
        a3 = -a1 * n1 / (N - n1 - n2)
        for i in range(1, N):
            if i < n1:
                qdd_ = np.append(qdd_, a1)
                qd_ = np.append(qd_, qd_[i - 1] + qdd_[i - 1] * ts)
                q_ = np.append(q_, q_[i - 1] + qd_[i - 1] * ts)
            elif i >= n1 and i < (n1 + n2):
                qdd_ = np.append(qdd_, 0)
                qd_ = np.append(qd_, qd_[i - 1] + qdd_[i - 1] * ts)
                q_ = np.append(q_, q_[i - 1] + qd_[i - 1] * ts)
            else:
                qdd_ = np.append(qdd_, a3)
                qd_ = np.append(qd_, qd_[i - 1] + qdd_[i - 1] * ts)
                q_ = np.append(q_, q_[i - 1] + qd_[i - 1] * ts)
        return q_, qd_, qdd_

    def plotTrajectory(self):
        time_slot = np.linspace(
            0.0, (np.sum(self.Nf) + 1) * self.ts, num=(np.sum(self.Nf) + 1)
        )
        fig, axs = plt.subplots(3, 1)
        for i in range(self.njoints):
            axs[0].plot(time_slot, self.q[i])
            axs[0].set_ylabel("q")
            axs[1].plot(time_slot, self.qd[i])
            axs[1].set_ylabel("qd")
            axs[2].plot(time_slot, self.qdd[i])
            axs[2].set_ylabel("qdd")
            axs[2].set_xlabel("Time(s)")
        x = [0, 10, 20, 30]
        for j in range(3):
            for xc in x:
                axs[j].axvline(x=xc, color="black", linestyle="dashed")
        
