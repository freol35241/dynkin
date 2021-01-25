import math
import numpy as np

from . import assert_3D_vector

def eulerian(attitude):
    """Returns eulerian matrix for specified attitude vector
    """
    attitude = assert_3D_vector(attitude)
    fi, theta, _ = attitude
    return np.array([
        [1,     math.sin(fi)*math.cos(theta),   math.cos(fi)*math.tan(theta)],
        [0,     math.cos(fi),                   -math.sin(fi)],
        [0,     math.sin(fi)/math.cos(theta),   math.cos(fi)/math.cos(theta)]
    ])
    
def euler_to_rotation(attitude) :
    """
    Returns rotation matrix defined by Euler angles in attitude, according to YPR-convention
    """
    
    R_x = np.array([[1,         0,                          0                   ],
                    [0,         math.cos(attitude[0]),      -math.sin(attitude[0]) ],
                    [0,         math.sin(attitude[0]),      math.cos(attitude[0])  ]
                    ])
                    
    R_y = np.array([[math.cos(attitude[1]),     0,      math.sin(attitude[1])  ],
                    [0,                         1,      0                   ],
                    [-math.sin(attitude[1]),    0,      math.cos(attitude[1])  ]
                    ])
                
    R_z = np.array([[math.cos(attitude[2]),     -math.sin(attitude[2]),    0],
                    [math.sin(attitude[2]),     math.cos(attitude[2]),     0],
                    [0,                         0,                          1]
                    ])

    return R_z @ R_y @ R_x

def rotation_to_euler(R) :
    """
    Extracts euler angles from the given rotation matrix R (3x3), according to YPR convention
    """
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6
    if not singular :
        fi = math.atan2(R[2,1] , R[2,2])
        theta = math.atan2(-R[2,0], sy)
        psi = math.atan2(R[1,0], R[0,0])
    else :
        fi = math.atan2(-R[1,2], R[1,1])
        theta = math.atan2(-R[2,0], sy)
        psi = 0

    return np.array([fi, theta, psi])

def angular_velocity_to_deuler(attitude, angular_velocity):
    """
    Returns derivative of euler angles (attitude) from angular velocity
    """
    a = assert_3D_vector(attitude)
    w = assert_3D_vector(angular_velocity)
    return eulerian(a) @ w
    