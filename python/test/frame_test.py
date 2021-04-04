import pytest
import numpy as np
from numpy.testing import assert_almost_equal

from dynkin import Frame, transform

    
def test_single_frame_position():
    
    f1 = Frame([1, 1, 1], np.deg2rad([90, 0, 90]))
    
    t = transform(None, f1)
    
    assert_almost_equal([1, 1, 1], t.apply_position([0, 0, 0]))
    
    assert_almost_equal([1, 2, 1], t.apply_position([1, 0, 0]))
    
    assert_almost_equal([1, 1, 2], t.apply_position([0, 1, 0]))
    
    assert_almost_equal([2, 1, 1], t.apply_position([0, 0, 1]))
    
def test_single_frame_vector():
    
    f1 = Frame([1, 1, 1], np.deg2rad([0, 90, 90]))
    
    t = transform(None, f1)
    
    assert_almost_equal([0, 0, 0], t.apply_vector([0, 0, 0]))
    
    assert_almost_equal([0, 0, -1], t.apply_vector([1, 0, 0]))
    
    assert_almost_equal([-1, 0, 0], t.apply_vector([0, 1, 0]))

    assert_almost_equal([0, 1, 0], t.apply_vector([0, 0, 1]))
    
def test_single_frame_wrench():
    
    f1 = Frame([1, 1, 1], np.deg2rad([0, 90, 90]))
    
    t = transform(None, f1)
    
    assert_almost_equal([0, 0, -1, -1, 1, 0], t.apply_wrench([1, 0, 0, 0, 0, 0]))
    
    assert_almost_equal([-1, 0, 0, 0, -1, 1], t.apply_wrench([0, 1, 0, 0, 0, 0]))
    
    assert_almost_equal([0, 1, 0, -1, 0, 1], t.apply_wrench([0, 0, 1, 0, 0, 0]))
    
@pytest.mark.skip("HTM attribute not exposed yet")
def test_single_frame_HTM_inverse():
    
    f1 = Frame([124, -343, -13], np.deg2rad([27, 49, 62]))
    
    t = transform(None, f1)
    
    t_ = transform(f1, None)
    
    assert_almost_equal(t._HTM, t_.inv().HTM)
    
def test_single_frame_velocity():
    
    f1 = Frame([1, 1, 1], np.deg2rad([0, 90, 90]))
    f1.linear_velocity = [1, 0, 0]
    f1.angular_velocity = [0, 0, 1]
    
    assert_almost_equal([1, 0, 0, 0, 0, 1], f1.get_twist())
    
def test_chained_frame_velocity():
    
    f1 = Frame(attitude=np.deg2rad([0, 0, 90]))
    f1.angular_velocity = [0, 0, 1]
    f2 = f1.align_child(position=[1, 1, 0], linear_velocity=[1, 0, 0], angular_velocity=[1, 0, 0])
    
    assert_almost_equal([0, 1, 0, 1, 0, 1], f2.get_twist())
    