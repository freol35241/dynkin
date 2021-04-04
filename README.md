# dynkin

A toolkit for 3D dynamics and kinematics of rigid bodies using the YPR euler angle convention. Written in C++ as a single-file, header-only library based on the Eigen vector library. Python bindings are built using pybind11 and available on PyPi.

## General

`dynkin` is a set of tools for handling the dynamics and kinematics of rigid bodies in 3 dimensions (6DOFs). Some features:

* Homogenous transformation matrices
* Chained reference frames
* Idealized rigid body implementation

The fundamentals of reference frames and the kinematic relations of these are based on [Theory of Applied Robotics (Reza N. Jazar)](https://link.springer.com/book/10.1007/978-0-387-68964-7) , the idealized rigid body implementation follows the outline suggested in the lectures by [Fossen](https://www.fossen.biz/wiley/ed2/Ch3.pdf).


### Theory intro

Some basic notions:

* A reference `Frame` is defined in `dynkin` as an object which `positions`, `vectors`, `velocities`, `accelerations` etc can be decomposed in. `dynkin` represents reference `Frame`s by (4x4) homogenous transformation matrices. A `Frame` is aligned (positioned, rotated) and moves (linear and angular velocity) in relation to another `Frame` or the inertial frame (represented by `None`).
* The `pose` of a `Frame` is its generalized position and the `twist` of a `Frame` is its generalized velocity, both in relation to the inertial frame.
* All rotations in `dynkin` are internally represented by rotation matrices but the external API, so far, deals only with Euler angles of the YPR (Yaw-Pitch-Roll) convention.
* A `kinematic chain` is a single-linked list of `Frame`s, where each `Frame` holds a reference to its closest parent. Any number of `Frame`s may be attached into such a chain and the chain may also have any number of branches, it is however the user´s responsibility to ensure no kinematic loops occur.
* A `transform` is an object relating two `Frame`s enabling transformation of `positions`, `vectors`, `velocities` etc from one `Frame` to the other. The `Frame`s do not need to be part of the same `kinematic chain`.
* A `RigidBody` is a 3D body with arbitrary extent that may be described by a generalized inertia matrix (6x6). It accelerates when subject to generalized external forces (`wrenches`) and rotational velocities give rise to inertial forces (coriolis and centripetal contributions).

## Installation

### C++

`dynkin` can be included as per below in your cmake workflow if using the [CPM.cmake package manager](https://github.com/TheLartians/CPM.cmake):

```cmake
CPMAddPackage(
  NAME dynkin
  GITHUB_REPOSITORY freol35241/dynkin
  VERSION 0.1.0
)
```

Or, since `dynkin` is a single-file and header-only file library, just copy ```dynkin.hpp``` into your project.

### Python

Available on PyPi:

```
pip install dynkin
```

## Examples

### C++

**Single frame**

```c++
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

#define DEG2RAD M_PI/180.0f

using namespace dynkin;

Frame frame1 = create_frame();
frame1->position() << 1, 2, 3;
frame1->set_attitude({0,0,90*DEG2RAD});

// Find the transformation that takes the inertial frame into frame1
Transform ti1 = transform(nullptr, frame1);

// Transformation of vector
Eigen::Vector3d v1_decomposed_in_inertial_frame, v1_decomposed_in_frame1;
v1_decomposed_in_inertial_frame = ti1.apply_vector(v1_decomposed_in_frame1);

// Transformation of position
Eigen::Vector3d p1_decomposed_in_inertial_frame, p1_decomposed_in_frame1;
p1_decomposed_in_inertial_frame = ti1.apply_position(p1_decomposed_in_frame1);

// Transformation of wrench
Eigen::Vector6d w1_decomposed_in_inertial_frame, w1_decomposed_in_frame1;
w1_decomposed_in_inertial_frame = ti1.apply_wrench(w1_decomposed_in_frame1);

// Find the inverse transformation
Transform t1i = ti1.inverse();

// Pose of this frame, decomposed in inertial frame
Eigen::Vector6d pose = frame1.get_pose();

// Twist of this frame, decomposed in inertial frame
Eigen::Vector6d = frame1.get_twist();
```

**Two frames**
```c++
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

#define DEG2RAD M_PI/180.0f

using namespace dynkin;

Frame frame1 = create_frame();
frame1->position() << 1, 2, 3;
frame1->set_attitude({0,0,90*DEG2RAD});

Frame frame2 = create_frame();
frame2->position() << 3, 2, 1;
frame2->set_attitude({0,0,-90*DEG2RAD});

// Find transformation taking frame1 into frame2
Transform t12 = transform(frame1, frame2);

// Transformation of vector
Eigen::Vector3d v1_decomposed_in_frame1, v1_decomposed_in_frame2;
v1_decomposed_in_frame1 = t12.apply_vector(v1_decomposed_in_frame2);

// Transformation of position
Eigen::Vector3d p1_decomposed_in_frame1, p1_decomposed_in_frame2;
p1_decomposed_in_frame1 = t12.apply_position(p1_decomposed_in_frame2);

// Transformation of wrench
Eigen::Vector6d w1_decomposed_in_frame1, w1_decomposed_in_frame2;
w1_decomposed_in_frame1 = t12.apply_wrench(w1_decomposed_in_frame2);

// Find the inverse transformation
Transform t21 = t12.inverse();
```

**Kinematic chains**
```c++
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

#define DEG2RAD M_PI/180.0f

using namespace dynkin;

Frame frame1 = create_frame();
frame1->position() << 1, 2, 3;
frame1->set_attitude({0,0,90*DEG2RAD});

Frame frame2 = frame1->create_child();
frame1->position() << 3, 2, 1;
frame1->set_attitude({0,0,-90*DEG2RAD});

Frame frame3 = frame2->create_child();
frame3->position() << 1, 1, 1;


// Find transformation taking the inertial frame into frame3
Transform ti3 = transform(None, frame3);

// Find transformation taking frame3 into frame1
Transform t31 = transform(frame3, frame1);

...
```

**Rigid body**
```c++
TODO
```

### Python

**Single frame**
```python
import numpy as np # Not a dependency of dynkin!
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=np.deg2rad([0, 0, 90]))

# Find the transformation that takes the inertial frame into frame1
ti1 = transform(None, frame1)

# Transformation of vector
v1_decomposed_in_inertial_frame = ti1.apply_vector(v1_decomposed_in_frame1)

# Transformation of position
p1_decomposed_in_inertial_frame = ti1.apply_position(p1_decomposed_in_frame1)

# Transformation of wrench
w1_decomposed_in_inertial_frame = ti1.apply_wrench(w1_decomposed_in_frame1)

# Find the inverse transformation
t1i = ti1.inv()

# Pose of this frame, decomposed in inertial frame
frame1.get_pose()

# Twist of this frame, decomposed in inertial frame
frame.get_twist()
```

**Two frames**
```python
import numpy as np # Not a dependency of dynkin!
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=np.deg2rad([0, 0, 90]))
frame2 = Frame(position=[3, 2, 1], attitude=np.deg2rad([0, 0, -90]))

# Find transformation taking frame1 into frame2
t12 = transform(frame1, frame2)

# Transformation of vector
v1_decomposed_in_frame1 = t12.apply_vector(v1_decomposed_in_frame2)

# Transformation of position
p1_decomposed_in_frame1 = t12.apply_position(p1_decomposed_in_frame2)

# Transformation of wrench
w1_decomposed_in_frame1 = t12.apply_wrench(w1_decomposed_in_frame2)

# Find the inverse transformation
t21 = t12.inv()
```

**Kinematic chains**
```python
import numpy as np # Not a dependency of dynkin!
from dynkin import Frame, transform

frame1 = Frame(position=[1, 2, 3], attitude=np.deg2rad([0, 0, 90]))
frame2 = frame1.align_child(position=[3, 2, 1], attitude=np.deg2rad([0, 0, -90]))
frame3 = frame2.align_child(position=[1, 1, 1], attitude=[0, 0, 0])

# Find transformation taking the inertial frame into frame3
ti3 = transform(None, frame3)

# Find transformation taking frame3 into frame1
t31 = transform(frame3, frame1)

...
```

**Rigid body**
```python
TODO
```


## API documentation

### C++

https://freol35241.github.io/dynkin/

### Python

TODO

## License

Distributed under the terms of the MIT license, `dynkin` is open source software.

