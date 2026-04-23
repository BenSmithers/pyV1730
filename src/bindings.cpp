#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "digitizer.hpp"

namespace py = pybind11;

PYBIND11_MODULE(caen_v1730, m) {
    py::class_<V1730Digitizer>(m, "V1730Digitizer")
        .def(py::init<int, int, int, uint32_t>(),
             py::arg("link_type"),
             py::arg("link_num"),
             py::arg("conet_node"),
             py::arg("base_addr"))

        .def("configure_external_trigger", &V1730Digitizer::configure_external_trigger)
        .def("start_acquisition", &V1730Digitizer::start_acquisition)
        .def("stop_acquisition", &V1730Digitizer::stop_acquisition)
        .def("read_waveforms", &V1730Digitizer::read_waveforms)
        .def("acquire_multiple", &V1730Digitizer::acquire_multiple)
        .def("count_hits", &V1730Digitizer::count_hits)
        .def("set_threshold", &V1730Digitizer::set_threshold)
        .def("pull_charge",&V1730Digitizer::pull_charge)
        .def("set_record_length", &V1730Digitizer::set_record_length)
        .def("set_post_trig_size", &V1730Digitizer::set_post_trig_size);
}