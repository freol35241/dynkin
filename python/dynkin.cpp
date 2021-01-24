#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>

namespace py = pybind11;

Eigen::Vector3d elementwise_sum(const Eigen::Vector3d& first, const Eigen::Vector3d& second){
    return first + second;
}

PYBIND11_MODULE(pydynkin, m) {
    m.doc() = "dynkin!";

    m.def("sum", &elementwise_sum, "A function which adds two 3D vectors");
}