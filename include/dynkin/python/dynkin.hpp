#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace dynkin;
using namespace dynkin::rigidbody;

namespace dynkin{
    namespace python{

        inline void include_dynkin_pybind_wrappers(py::module& m){
            m.doc() = R"docs(
                A toolkit for 3D dynamics and kinematics of rigid bodies using the
                YPR euler angle convention.
            )docs";

            py::class_<_Frame, Frame>(m, "Frame")
                .def(py::init(
                    [](
                        const Eigen::Vector3d& position,
                        const Eigen::Vector3d& attitude,
                        const Eigen::Vector3d& linear_velocity,
                        const Eigen::Vector3d& angular_velocity,
                        Frame parent
                    ){
                        Frame out = create_frame(parent);
                        out->position() = position;
                        out->set_attitude(attitude);
                        out->linear_velocity() = linear_velocity;
                        out->angular_velocity() = angular_velocity;
                        return out;
                    }),
                    "position"_a = Eigen::Vector3d::Zero(),
                    "attitude"_a = Eigen::Vector3d::Zero(),
                    "linear_velocity"_a = Eigen::Vector3d::Zero(),
                    "angular_velocity"_a = Eigen::Vector3d::Zero(),
                    "parent"_a = nullptr
                )
                .def("align_child",
                    [](
                        _Frame& self,
                        const Eigen::Vector3d& position,
                        const Eigen::Vector3d& attitude,
                        const Eigen::Vector3d& linear_velocity,
                        const Eigen::Vector3d& angular_velocity
                    ){
                        Frame out = self.create_child();
                        out->position() = position;
                        out->set_attitude(attitude);
                        out->linear_velocity() = linear_velocity;
                        out->angular_velocity() = angular_velocity;
                        return out;
                    },
                    "position"_a = Eigen::Vector3d::Zero(),
                    "attitude"_a = Eigen::Vector3d::Zero(),
                    "linear_velocity"_a = Eigen::Vector3d::Zero(),
                    "angular_velocity"_a = Eigen::Vector3d::Zero()
                )
                .def("get_pose", &_Frame::get_pose)
                .def("get_twist", &_Frame::get_twist)
                .def_property("position",
                    &_Frame::position,
                    [](_Frame& self, const Eigen::Vector3d& value){
                        self.position() = value;
                    }
                )
                .def_property("attitude",
                    &_Frame::get_attitude,
                    &_Frame::set_attitude
                )
                .def_property("linear_velocity",
                    &_Frame::linear_velocity,
                    [](_Frame& self, const Eigen::Vector3d& value){
                        self.linear_velocity() = value;
                    }
                )
                .def_property("angular_velocity",
                    &_Frame::angular_velocity,
                    [](_Frame& self, const Eigen::Vector3d& value){
                        self.angular_velocity() = value;
                    }
                );

            m.def("transform", &transform, "Create a Transform between two Frames");

            py::class_<Transform>(m, "Transform")
                .def("apply_vector", &Transform::apply_vector)
                .def("apply_position", &Transform::apply_position)
                .def("apply_wrench", &Transform::apply_wrench)
                .def("inverse", &Transform::inverse);


            py::module rb = m.def_submodule("rigidbody");

            py::class_<RigidBody>(rb, "RigidBody")
                .def(py::init(
                    [](
                        const double& mass,
                        const Eigen::Vector3d& gyradii,
                        const Eigen::Vector3d& cog
                    ){
                        return RigidBody(
                            generalized_inertia_matrix(mass, gyradii),
                            cog
                        );
                    }),
                    "mass"_a,
                    "gyradii"_a,
                    "cog"_a = Eigen::Vector3d::Zero()        
                )
                .def_readonly("origin", &RigidBody::origin)
                .def_readonly("CoG", &RigidBody::CoG)
                .def("generalized_coordinates", &RigidBody::generalized_coordinates)
                .def("generalized_velocities", &RigidBody::generalized_velocities)
                .def("acceleration", &RigidBody::acceleration,
                    "wrench"_a,
                    "additional_inertia"_a = Eigen::Matrix6d::Zero())
                .def("wrench", &RigidBody::wrench,
                    "acceleration"_a,
                    "additional_inertia"_a = Eigen::Matrix6d::Zero()
                );
        }

    }
}
