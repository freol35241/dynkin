"""
A toolkit for 3D dynamics and kinematics of rigid bodies using the
 YPR euler angle convention.

.. include:: ../README.md
"""

__pdoc__ = {
    '': False,
    'euler': False,
    'frame': False,
    'rigid_body': False,
}

import numpy as np

def assert_3D_vector(v):
    """
    Make sure v is a 3x1 numpy array with dtype==float
    """
    out = np.array(v, dtype=float)
    if not out.size == 3:
        raise AssertionError('Must be a np.ndarray of size 3!')
    return out

def assert_6D_vector(v):
    """
    Make sure v is a 6x1 numpy array with dtype==float
    """
    out = np.array(v, dtype=float)
    if not out.size == 6:
        raise AssertionError('Must be a np.ndarray of size 6!')
    return out

def assert_6D_matrix(m):
    """
    Make sure m is a 6x6 numpy array with dtype==float
    """
    out = np.array(m, dtype=float)
    if not out.shape == (6, 6):
        raise ValueError('Must be a np.ndarray of shape (6,6)!')
    return out


from .frame import Frame, Transform, transform
from .rigid_body import RigidBody

__all__ = [
    'RigidBody',
    'Frame',
    'Transform',
    'transform'
]