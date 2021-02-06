import numpy as np

from . import assert_3D_vector, assert_6D_vector
from .euler import euler_to_rotation, rotation_to_euler

class Frame:
    """
    A coordinate reference frame
    """    
    def __init__(self, position=None, attitude=None, linear_velocity=None, angular_velocity=None, degrees=False, parent=None):
        self._parent = parent
        self._HTM = np.eye(4)
        
        self.set_position(position or [0, 0, 0])
        self.set_attitude(attitude or [0, 0, 0,], degrees=degrees)
        self.set_linear_velocity(linear_velocity or [0, 0, 0])
        self.set_angular_velocity(angular_velocity or [0, 0, 0], degrees=degrees)
        
    @property
    def parent(self):
        """
        Reference to parent Frame
        """
        return self._parent
        
    @property
    def HTM(self):
        """
        Homogenous Transformation Matrix (4x4) defining this Frame
        """
        return self._HTM
    
    @property
    def position(self):
        """
        Position (3x1) of this Frame in relation to parent Frame
        """
        return self._HTM[:3, -1]
        
    def set_position(self, position):
        """
        Set position (3x1) of this Frame in relation to parent Frame
        """
        self._HTM[:3, -1] = assert_3D_vector(position)
        
    @property
    def rotation(self):
        """
        Rotation (3x3) of this Frame in relation to parent Frame
        """
        return self.HTM[:3, :3]
    
    def set_rotation(self, rotation):
        """
        Set rotation (3x3) of this Frame in relation to parent Frame
        """
        self._HTM[:3, :3] = rotation
        
    @property
    def attitude(self):
        """
        Attitude (euler angles)(3x1) of this Frame in relation to parent Frame
        """
        return rotation_to_euler(self.rotation)
        
    def set_attitude(self, attitude, degrees=False):
        """
        Set attitude (euler angles)(3x1) of this Frame in relation to parent Frame
        """
        self._HTM[:3, :3] = euler_to_rotation(
            assert_3D_vector(np.deg2rad(attitude) if degrees else attitude)
        )
            
    @property
    def linear_velocity(self):
        """
        Linear velocity (3x1) of this Frame in relation to parent Frame, decomposed in this Frame
        """
        return self._linear_velocity
    
    def set_linear_velocity(self, velocity):
        """
        Set linear velocity (3x1) of this Frame in relation to parent Frame, decomposed in this Frame
        """
        self._linear_velocity = assert_3D_vector(velocity)
            
    @property
    def angular_velocity(self):
        """
        Angular velocity (3x1) of this Frame in relation to parent Frame, decomposed in this Frame
        """
        return self._omega
    
    def set_angular_velocity(self, omega, degrees=False):
        """
        Set angular velocity (3x1) of this Frame in relation to parent Frame, decomposed in this Frame
        """
        self._omega = assert_3D_vector(np.deg2rad(omega) if degrees else omega)
        
    def get_pose(self):
        """
        Pose (6x1) of this Frame in relation to the inertial Frame
        """
        t = transform(None, self)
        return np.array([
            *t.HTM[:3, -1],
            *rotation_to_euler(t.HTM[:3, :3])
        ])
        
    def get_twist(self):
        """
        Twist (6x1) of this Frame in relation to the inertial Frame
        """
        v = self.linear_velocity
        w = self.angular_velocity
        
        if self.parent is not None:
            twist_p = self.parent.get_twist()
            t = transform(self, self.parent)
            v += t.apply_vector(twist_p[:3] + np.cross(twist_p[3:], self.position))
            w += t.apply_vector(twist_p[3:])
            
        return np.array([*v, *w])            
        
    def align_child(self, *args, **kwargs):
        """
        Align a child frame in relation to this Frame, accepts same args and kwargs as Frame()
        """
        kwargs.update({
            'parent': self
        })
        return Frame(*args, **kwargs)

def transform(zeroth_frame, end_frame):
    """
    Finds a Transform between two frames
    """    
    HTM = np.eye(4)
    f = end_frame
    
    while f:
        if f == zeroth_frame:
            return Transform(HTM)
        
        HTM = f.HTM @ HTM
        f = f.parent
        
    # If we get to here, the least common base fram is the inertial frame (None)
    T = Transform(HTM)
    
    if zeroth_frame is None:
        return T
    
    T_ = transform(None, zeroth_frame).inv()
    
    return Transform(T_.HTM @ T.HTM)

class Transform:
    """
    A Transform to be applied to positions, vectors, wrenches
    """    
    def __init__(self, HTM):
        self._HTM = HTM
        
    @property
    def translation(self):
        """
        Translation part (3x1) of Transform
        """
        return self._HTM[:3, -1]
    
    @property
    def HTM(self):
        """
        Homogenous Transformation Matrix (4x4) of Transform
        """
        return self._HTM
    
    def apply_vector(self, vector):
        """
        Applies this Transform to vector
        """
        vector = assert_3D_vector(vector)
        return self._HTM[:3, :3] @ vector
    
    def apply_position(self, position):
        """
        Applies this Transform to position
        """
        tmp = np.ones(4)
        tmp[:3] = assert_3D_vector(position)
        return (self._HTM @ tmp)[:3]
    
    def apply_wrench(self, wrench):
        """
        Applies this Transform to wrench
        """
        wrench = assert_6D_vector(wrench)
        out = wrench[:]
        out[:3] = self.apply_vector(wrench[:3])
        out[3:] = self.apply_vector(wrench[3:]) + np.cross(self.translation, out[:3])
        return out
        
    def inv(self):
        """
        Returns inverse of this Transform
        """
        HTM_ = np.linalg.inv(self._HTM)
        return Transform(HTM_)
