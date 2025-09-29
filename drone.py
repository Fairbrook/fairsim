import rerun as rr
import numpy as np
import math
import time


class Quaternion:
    #       w, x, y, z
    data = np.array([1, 0, 0, 0])

    def __init__(self, data=None):
        if data is not None:
            self.data = data

    def __getitem__(self, key):
        return self.data[key]

    def __mul__(self, r):
        if isinstance(r, float) or isinstance(r, int):
            return Quaternion(self.data * r)
        result = np.array([
            self.w()*r.w() - self.x()*r.x() - self.y()*r.y() - self.z()*r.z(),
            self.w()*r.x() + self.x()*r.w() + self.y()*r.z() - self.z()*r.y(),
            self.w()*r.y() - self.x()*r.z() + self.y()*r.w() + self.z()*r.x(),
            self.w()*r.z() + self.x()*r.y() - self.y()*r.x() + self.z()*r.w(),
        ])
        return Quaternion(result)

    def __add__(self, r):
        result = np.array([
            self.w() + r.w(),
            self.x() + r.x(),
            self.y() + r.y(),
            self.z() + r.z(),
        ])
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
    pos = np.array([0.0, 0.0, 0.0])
    g = np.array([0, 0, -9.81])
    pdot = np.array([0.0, 0.0, 0.0])
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

    omega = np.array([0.0, 0.0, 0.0])
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
        return -np.dot(self.kpt, (self.pos-pdv)) \
            - np.dot(self.kdt, (self.pdot)) \
            - self.m*self.g

    def get_torque_input(self, fu, yaw):
        unitz = np.array([0, 0, 1])
        fu_unit = fu/np.linalg.norm(fu)
        qz = Quaternion(np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))
        dot = np.dot(fu_unit, unitz)
        cross = np.linalg.cross(fu_unit, unitz)
        if (np.linalg.norm(cross) == 0):
            imaginary = [0, 0, 0]
        else:
            imaginary = (cross/np.linalg.norm(cross))*math.sqrt((1-dot)/2)
        real = math.sqrt((1+dot)/2)
        if 2*np.acos(real) > math.pi:
            q = Quaternion(
                np.array([-real, imaginary[0], imaginary[1], imaginary[2]]))
        else:
            q = Quaternion(
                np.array([real, -imaginary[0], -imaginary[1], -imaginary[2]]))
        q = qz.conjugate() * q * self.q
        return -2*np.dot(self.kpr, q.ln())\
            - np.dot(self.kdr, self.omega)\
            + np.cross(self.omega, np.dot(self.J, self.omega))

    def update_state(self, position, yaw, dt):
        f = self.get_f_desired(position)
        tau = self.get_torque_input(f, yaw)
        self.pos += self.pdot * dt
        self.pdot += (self.q *
                      Quaternion([0, *f/self.m]) * self.q.conjugate() +
                      Quaternion([0, *self.g])).data[1:] * dt
        self.q = self.q + self.q*Quaternion([0, *self.omega])*0.5*dt
        self.omega += np.dot(np.linalg.inv(self.J), tau -
                             np.cross(self.omega, np.dot(self.J, self.omega))) * dt


rr.init("drone", spawn=True)

rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP)
rr.log("world/drone", rr.Asset3D(path="./X500.glb"))

T = 10
dt = 0.01
t = 0
drone = Drone()

while t < T:
    drone.update_state([3, 3, 3], 5, dt)
    rr.log("world/drone",
           rr.Transform3D(translation=drone.pos, quaternion=[*drone.q[1:], drone.q[0]]))
    time.sleep(0.1)
    t += dt
