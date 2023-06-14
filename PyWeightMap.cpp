#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "WeightMap.hpp"
#include <cstdio>
#include <string>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(PyWeightMap, m) {
    m.doc() = "Python Bindings to WeightMap class"; // optional module docstring

    py::class_<WeightMap>(m, "PyWeightMap")
        .def(py::init<const mapsize_t, const mapsize_t>())
        //Data
        .def_property_readonly("width", &WeightMap::getWidth)
        .def_property_readonly("height", &WeightMap::getHeight)
        .def_property_readonly("max", &WeightMap::getMaxWeightInMap)
        
        //Static Data
        .def_property_readonly_static("maxWeight", [](py::object /* self */){ return WeightMap::getMaxWeight(); }) //Going through lambda is necessary if it does not take arguements
        .def_property_readonly_static("minWeight", [](py::object /* self */){ return WeightMap::getMinWeight(); })

        //Weight setter and getter
        .def("getWeight", &WeightMap::getWeight)
        .def("setWeight", &WeightMap::setWeight)

        //PathPlanning
        .def("getPath", &WeightMap::getPath)
        .def("pathToXVal", &WeightMap::pathToXVal)

        //Path Utility
        .def_static("compressPath", [](WeightMap::path_t path){ 
            WeightMap::compressPath(path); 
            return path;
            }
        )
        .def_static("is_path_continuous", &WeightMap::is_path_continuous)
        .def_static("smooth_path", [](WeightMap::path_t path, float smoothing_val){ 
            WeightMap::smoothPath(path, smoothing_val); 
            return path;
            })

        //AddBoarder
        .def_property_readonly_static("border_top", [](py::object /* self */){return BoarderPlace::TOP.get_value();})
        .def_property_readonly_static("border_bottom", [](py::object /* self */){return BoarderPlace::BOTTOM.get_value();})
        .def_property_readonly_static("border_right", [](py::object /* self */){return BoarderPlace::RIGHT.get_value();})
        .def_property_readonly_static("border_left", [](py::object /* self */){return BoarderPlace::LEFT.get_value();})
        .def("add_border", [](WeightMap& self, mapsize_t boarder_width, weight_t boarder_weight, int place){return self.addBoarder(boarder_width, boarder_weight, BoarderPlace(place));})

        .def("addObstical", &WeightMap::addObstical)

        .def("isValidPoint", &WeightMap::isValidPoint)

        //Python Dunder Methods
        .def("__repr__",
            [](const WeightMap &self) {
                std::string str(255, '\0');
                snprintf((char*)str.c_str(), str.size() * sizeof(str[0]), "<PyWeightMap.PyWeightMap at '%p'>", &self);

                return str;
            }
        )
        .def("__str__",
            [](const WeightMap &self) {
                return self.to_string(false);
            }
        );
}