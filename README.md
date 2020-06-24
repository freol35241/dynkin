# dynkin

[![PyPI version shields.io](https://img.shields.io/pypi/v/dynkin.svg)](https://pypi.python.org/pypi/dynkin/)
![](https://github.com/freol35241/dynkin/workflows/dynkin/badge.svg)
[![codecov](https://codecov.io/gh/freol35241/dynkin/branch/master/graph/badge.svg)](https://codecov.io/gh/freol35241/dynkin)

A toolkit for 3D dynamics and kinematics of rigid bodies using the YPR euler angle convention.


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
* All rotations in `dynkin` are internally represented by rotation matrices but the external API, so far, deals only with Euler angles of the YPR (Yaw-Pitch-Roll) convention.
* A `kinematic chain` is a single-linked list of `Frame`s, where each `Frame` holds a reference to its closest parent. Any number of `Frame`s may be attached into such a chain and the chain may also have any number of branches, it is however the user´s responsibility to ensure no kinematic loops occur.
* A `transform` is an object relating two `Frame`s enabling transformation of `positions`, `vectors`, `velocities` etc from one `Frame` to the other. The `Frame`s do not need to be part of the same `kinematic chain`.
* A `RigidBody` is a 3D body with arbitrary extent that may be described by a generalized inertia matrix (6x6). It accelerates when exposed to generalized external forces (`wrenches`) and rotational velocities give rise to inertial forces (coriolis and centripetal contributions).

## Example usage

TODO


## License

Distributed under the terms of the MIT license, `dynkin` is free and open source software


