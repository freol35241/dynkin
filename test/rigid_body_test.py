import numpy as np
from numpy.testing import assert_almost_equal

from dynkin import RigidBody

def test_surge():
    rb = RigidBody(mass=1, gyradius=[1,1,1])
    print(rb.acceleration([1, 0, 0, 0, 0, 0]))
    
if __name__ == '__main__':
    test_surge()