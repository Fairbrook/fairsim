import rerun as rr
import numpy as np
import math


class Quaternion:
    #       w, x, y, z
    data = np.array([1, 0, 0, 0])

    def __init__(self, data=None):
        if data is not None:
            self.data = data

    def __getitem__(self, key):
        return self.data[key]

    def __mul__(self, r):
        result = [
            self.w()*r.w() - self.x()*r.x() - self.y()*r.y() - self.z()*r.z(),
            self.w()*r.x() + self.x()*r.w() + self.y()*r.z() - self.z()*r.y(),
            self.w()*r.y() - self.x()*r.z() + self.y()*r.w() + self.z()*r.x(),
            self.w()*r.z() + self.x()*r.y() - self.y()*r.x() + self.z()*r.w(),
        ]
        return Quaternion(result)

    def w(self):
        return self.data[0]

    def x(self):
        return self.data[1]

    def y(self):
        return self.data[2]

    def z(self):
        return self.data[3]

    def conjugate(self):
        return Quaternion([self.w(), -self.x(), -self.y(), -self.z()])

    def __str__(self):
        return f"w:{self.w():.3f} x:{self.x():.3f} y:{self.y():.3f} z:{self.z():.3f}"

    def ln(self):
        norm = np.linalg.norm(self.data[1:])
        if norm == 0:
            return np.array([0, 0, 0])

        return (self.data[1:]/norm)*math.acos(self.data[0])


class Drone:
    f = np.array([0, 0, 0])
    m = 1.5
    q = Quaternion()
    nz = np.array([0, 0, 1])
    pos = np.array([0, 0, 0])
    g = np.array([0, 0, 9.81])
    pdot = np.array([0, 0, 0])
    kpt = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    kdt = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    omega = np.array([0, 0, 0])
    J = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    kpr = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    kdr = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    def get_q_desired(self, fu, yaw=0):
        fua = np.array(fu)
        nu = fua / np.linalg.norm(fua)
        dot = np.dot(nu, self.nz)
        real = math.sqrt((1+dot)/2)
        cross = np.cross(nu, self.nz)
        imaginary = (cross/np.linalg.norm(cross)) * math.sqrt((1-dot)/2)
        return Quaternion(np.array([real, *imaginary])) * Quaternion(np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))

    def get_q_error(self, qd: Quaternion):
        return qd.conjugate()*self.q

    def get_f_desired(self, pd):
        pdv = np.array(pd)
        return -self.kpt*(self.pos-pdv) - self.kdt*(self.pdot) - self.m*self.g

    def get_torque_input(self, fu, yaw):
        unitz = np.array([0, 0, 1])
        fu_unit = np.array(fu)/np.linalg(np.array(fu))
        qz = Quaternion(np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))
        dot = np.dot(fu_unit, unitz)
        cross = np.linalg.cross(fu_unit, unitz)
        return -2*self.kpr*(qz*(math.sqrt((1+dot)/2)-(cross/np.linalg(cross))*math.sqrt((1-dot)/2)) * self.q).ln() - self.kdr*self.omega + np.cross(self.omega, self.J*self.omega)

    def update_state(self, torque):
        pass


rr.init("drone", spawn=True)

rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP)
rr.log("world/drone", rr.Asset3D(path="./X500.glb"))

drone = Drone()
qa = drone.get_q_desired([1, 1, 1])
print(qa)

rr.log("world/drone",
       rr.Transform3D(translation=[1, 1, 1], quaternion=[qa.x(), qa.y(), qa.z(), qa.w()]))
