import numpy as np
from numpy.testing import assert_almost_equal

from dynkin import Frame, transform

def test_single_frame_translation():
    
    f1 = Frame([1, 1, 1], [0, 0, 90], degrees=True)
    
    t = transform(None, f1)
    
    assert_almost_equal([1,1,1], t.translation)
    
    t_ = transform(f1, None)
    
    assert_almost_equal([-1,1,-1], t_.translation)
    
def test_single_frame_position():
    
    f1 = Frame([1, 1, 1], [90, 0, 90], degrees=True)
    
    t = transform(None, f1)
    
    assert_almost_equal([1, 1, 1], t.apply_position([0, 0, 0]))
    
    assert_almost_equal([1, 2, 1], t.apply_position([1, 0, 0]))
    
    assert_almost_equal([1, 1, 2], t.apply_position([0, 1, 0]))
    
    assert_almost_equal([2, 1, 1], t.apply_position([0, 0, 1]))
    
def test_single_frame_vector():
    
    f1 = Frame([1, 1, 1], [0, 90, 90], degrees=True)
    
    t = transform(None, f1)
    
    assert_almost_equal([0, 0, 0], t.apply_vector([0, 0, 0]))
    
    assert_almost_equal([0, 0, -1], t.apply_vector([1, 0, 0]))
    
    assert_almost_equal([-1, 0, 0], t.apply_vector([0, 1, 0]))

    assert_almost_equal([0, 1, 0], t.apply_vector([0, 0, 1]))
    
def test_single_frame_wrench():
    
    f1 = Frame([1, 1, 1], [0, 90, 90], degrees=True)
    
    t = transform(None, f1)
    
    assert_almost_equal([0, 0, -1, -1, 1, 0], t.apply_wrench([1, 0, 0, 0, 0, 0]))
    
    assert_almost_equal([-1, 0, 0, 0, -1, 1], t.apply_wrench([0, 1, 0, 0, 0, 0]))
    
    assert_almost_equal([0, 1, 0, -1, 0, 1], t.apply_wrench([0, 0, 1, 0, 0, 0]))
    
def test_single_frame_HTM_inverse():
    
    f1 = Frame([124, -343, -13], [27, 49, 62], degrees=True)
    
    t = transform(None, f1)
    
    t_ = transform(f1, None)
    
    assert_almost_equal(t._HTM, t_.inv().HTM)
    
def test_single_frame_velocity():
    
    f1 = Frame([1, 1, 1], [0, 90, 90], degrees=True)
    f1.set_linear_velocity([1, 0, 0])
    f1.set_angular_velocity([0, 0, 1])
    
    assert_almost_equal([1, 0, 0, 0, 0, 1], f1.get_twist())
    
def test_chained_frame_velocity():
    
    f1 = Frame(attitude=[0, 0, 90], degrees=True)
    f1.set_angular_velocity([0, 0, 1])
    f2 = f1.align_child(position=[1, 1, 0], linear_velocity=[1, 0, 0], angular_velocity=[1, 0, 0])
    
    assert_almost_equal([0, 1, 0, 1, 0, 1], f2.get_twist())
    