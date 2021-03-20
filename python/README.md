# dynkin

A toolkit for 3D dynamics and kinematics of rigid bodies using the YPR euler angle convention.

[**--> Docs <--**](https://freol35241.github.io/dynkin/)

**Note:** `dynkin` is also available in a C++ version, available here: https://github.com/freol35241/dynkin

## General

`dynkin` is a set of tools for handling the dynamics and kinematics of rigid bodies in 3 dimensions (6DOFs). Some features:

* Homogenous transformation matrices
* Chained reference frames
* Idealized rigid body implementation

The fundamentals of reference frames and the kinematic relations of these are based on [Theory of Applied Robotics (Reza N. Jazar)](https://link.springer.com/book/10.1007/978-0-387-68964-7) , the idealized rigid body implementation follows the outline suggested in the lectures by [Fossen](https://www.fossen.biz/wiley/ed2/Ch3.pdf).

## Installation

`pip install dynkin`

## Theory intro

Some basic notions:

* A reference `Frame` is defined in `dynkin` as an object which `positions`, `vectors`, `velocities`, `accelerations` etc can be decomposed in. `dynkin` represents reference `Frame`s by (4x4) homogenous transformation matrices. A `Frame` is aligned (positioned, rotated) and moves (linear and angular velocity) in relation to another `Frame` or the inertial frame (represented by `None`).
* The `pose` of a `Frame` is its generalized position and the `twist` of a `Frame` is its generalized velocity, both in relation to the inertial frame.
* All rotations in `dynkin` are internally represented by rotation matrices but the external API, so far, deals only with Euler angles of the YPR (Yaw-Pitch-Roll) convention.
* A `kinematic chain` is a single-linked list of `Frame`s, where each `Frame` holds a reference to its closest parent. Any number of `Frame`s may be attached into such a chain and the chain may also have any number of branches, it is however the user´s responsibility to ensure no kinematic loops occur.
* A `transform` is an object relating two `Frame`s enabling transformation of `positions`, `vectors`, `velocities` etc from one `Frame` to the other. The `Frame`s do not need to be part of the same `kinematic chain`.
* A `RigidBody` is a 3D body with arbitrary extent that may be described by a generalized inertia matrix (6x6). It accelerates when subject to generalized external forces (`wrenches`) and rotational velocities give rise to inertial forces (coriolis and centripetal contributions).

## Examples

### Single frame
```python
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=[0, 0, 90], degrees=True)

# Find transformation from the inertial frame to frame1
ti1 = transform(None, frame1)

# Transformation of vector
v1_decomposed_in_frame1 = ti1.apply_vector(v1_decomposed_in_inertial_frame)

# Transformation of position
p1_decomposed_in_frame1 = ti1.apply_position(p1_decomposed_in_inertial_frame)

# Transformation of wrench
w1_decomposed_in_frame1 = ti1.apply_wrench(w1_decomposed_in_inertial_frame)

# Find the inverse transformation
t1i = ti1.inv()

# Pose of this frame, decomposed in inertial frame
frame1.get_pose()

# Twist of this frame, decomposed in inertial frame
frame.get_twist()
```

### Two frames
```python
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=[0, 0, 90], degrees=True)
frame2 = Frame(position=[3, 2, 1], attitude=[0, 0, -90], degrees=True)

# Find transformation from frame1 to frame2
t12 = transform(frame1, frame2)

# Transformation of vector
v1_decomposed_in_frame2 = t12.apply_vector(v1_decomposed_in_frame1)

# Transformation of position
p1_decomposed_in_frame2 = t12.apply_position(p1_decomposed_in_frame1)

# Transformation of wrench
w1_decomposed_in_frame2 = t12.apply_wrench(w1_decomposed_in_frame1)

# Find the inverse transformation
t21 = t12.inv()
```

### Kinematic chains
```python
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=[0, 0, 90], degrees=True)
frame2 = frame1.align_child(position=[3, 2, 1], attitude=[0, 0, -90], degrees=True)
frame3 = frame2.align_child(position=[1, 1, 1], attitude=[0, 0, 0], degrees=True)

# Find transformation from inertial frame to frame3
ti3 = transform(None, frame3)

# Transformation from frame3 and frame1
t31 = transform(frame3, frame1)

...
```

TODO: RigidBody example

## License

Distributed under the terms of the MIT license, `dynkin` is free and open source software


