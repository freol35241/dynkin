#include <pybind11/pybind11.h>
#include <dynkin/python/dynkin.hpp>

PYBIND11_MODULE(_dynkin, m){
    dynkin::python::include_dynkin_pybind_wrappers(m);
}