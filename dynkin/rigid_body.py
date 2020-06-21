import numpy as np

from . import assert_3D_vector, assert_6D_vector, assert_6D_matrix
from .frame import Frame, transform
from .euler import angular_velocity_to_deuler

def skew(arr):
    arr = assert_3D_vector(arr)
    return np.array([[0,        -arr[2],    arr[1]],
                     [arr[2],   0,          -arr[0]],
                     [-arr[1],  arr[0],     0]])

class RigidBody(Frame):
    """
    Idealized rigid body implementation
    """
    def __init__(self, mass=None, gyradius=None, cog=None):
        super().__init__()
        
        assert(mass > 0)
        self._mass = mass
        
        gyradius = assert_3D_vector(gyradius)
        assert((gyradius > 0).all())
        self._gyradius = gyradius
        
        self._cog = self.align_child(
            position=cog
        )
        
    @property
    def mass(self):
        """
        Mass of this RigidBody
        """
        return self._mass
    
    @property
    def gyradius(self):
        """
        Gyradius of this RigidBody
        """
        return self._gyradius
    
    @property
    def CoG(self):
        """
        Reference to Centre of Gravity Frame of this RigidBody
        """
        return self._cog
    
    def motion_transformation_matrix(self):
        """
        Returns motion transformation matrix, H, (6x6)
        """
        H = np.eye(6)
        H[:3, 3:]= skew(self.CoG.position).T
        return H
    
    def coriolis_centripetal_matrix(self, inertia, twist):
        """
        Returns Coriolis-Centripetal matrix (6x6) of this RigidBody
        """
        inertia = assert_6D_matrix(inertia)
        twist = assert_6D_vector(twist)
        C = np.zeros((6,6))
        C[:3, :3] = self.mass*skew(twist[3:])
        C[3:, 3:] = -skew(inertia[3:, 3:] @ twist[3:])
        return C
    
    def generalized_inertia_matrix(self):
        """
        Returns generalized inertia matrix (6x6) of this RigidBody
        """
        M = np.zeros((6,6))
        M[:3, :3] = np.eye(3)*self.mass
        M[3:, 3:] = np.diag(self.mass*self.gyradius**2)
        return M
        
    def generalized_coordinates(self):
        """
        Returns the generalized coordinates of this RigidBody,
        [x, y, z, fi, theta, psi]
        """
        return self.get_pose()
    
    def generalized_velocities(self):
        """
        Returns the generalized velocities of this RigidBody,
        [dx, dy, dz, dfi, dtheta, dpsi]
        """
        twist = self.get_twist()
        t = transform(None, self)
        return np.array([
            *t.apply_vector(twist[:3]),
            *angular_velocity_to_deuler(self.attitude, twist[3:])
        ])

    def acceleration(self, wrench):
        """
        Returns the resulting acceleration from the given wrench
        """
        f = assert_6D_vector(wrench)
        twist = self.get_twist()
        H = self.motion_transformation_matrix()
        I_cg = self.generalized_inertia_matrix()
        C_cg = self.coriolis_centripetal_matrix(I_cg, twist)
        I_b = H @ I_cg @ H.T
        C_b = H @ C_cg @ H.T
        f_cc = C_b @ twist
        f -= f_cc
        
        return np.linalg.solve(I_b, f)
    
    def wrench(self, acceleration):
        """
        Returns the required wrench to obtain the given acceleration
        """
        a = assert_6D_vector(acceleration)
        twist = self.get_twist()
        H = self.motion_transformation_matrix()
        I_cg = self.generalized_inertia_matrix()
        C_cg = self.coriolis_centripetal_matrix(I_cg, twist)
        I_b = H @ I_cg @ H.T
        C_b = H @ C_cg @ H.T
        f_cc = C_b @ twist
        
        f = I_b @ a
        f -= f_cc
        
        return f