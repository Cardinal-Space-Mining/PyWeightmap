#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "WeightMap.hpp"
#include <cstdio>
#include <string>
#include <limits>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(PyWeightMap, m) {
    m.doc() = "Python Bindings to WeightMap class"; // optional module docstring

    //Path Utility
    m.def("compressPath", [](WeightMap::path_t path){ 
        WeightMap::compressPath(path); 
        return path;
        },
    "Compresses the path to the bare minimum required to represent the path. Eg, the path [(0,0), (1,1), (2,2)] is converted to the path [(0,0), (2,2)]", py::arg("path"));
    m.def("is_path_continuous", &WeightMap::is_path_continuous, "Returns true if the path is continuous, false otherwise", py::arg("path"));
    m.def("smooth_path", [](WeightMap::path_t path, float smoothing_val){ 
        WeightMap::smoothPath(path, smoothing_val); 
        return path;
    },
    "smooths the path. Smoothing is varried by changing smoothing value. The smoothing value can varry from 0 (very smooth) to 1 (no smoothing at all)", py::arg("path"), py::arg("smoothing_val"));

    auto py_wm = py::class_<WeightMap>(m, "WeightMap");
        
    py_wm.def(py::init<const mapsize_t, const mapsize_t>(),"Initializes the WeightMap to a given size", py::arg("width"), py::arg("height"));
    //Data
    py_wm.def_property_readonly("width", &WeightMap::getWidth, "width of the WeightMap in cells");
    py_wm.def_property_readonly("height", &WeightMap::getHeight, "height of the WeightMap in cells");
    py_wm.def_property_readonly("max_value", &WeightMap::getMaxWeightInMap, "the maximum weight in the WeightMap");
    
    //Static Data
    py_wm.def_property_readonly_static("maxWeight", [](py::object /* self */){ return WeightMap::getMaxWeight(); }, "the maximum weight that a WeightMap can hold"); //Going through lambda is necessary if it does not take arguements
    py_wm.def_property_readonly_static("minWeight", [](py::object /* self */){ return WeightMap::getMinWeight(); }, "the minimum weight a WeightMap can hold");

    //Weight setter and getter
    py_wm.def("getWeight", &WeightMap::getWeight, "Returns the weight at the provided location.\nValid values for x are [0, self.width-1)\nValid values for y [0, self.height-1)", py::arg("x"), py::arg("y"));
    py_wm.def("setWeight", &WeightMap::setWeight, "Sets the weight of at provided location.\nValid values for x are x in range(0, self.width-1)\nValid values for y are y in range(0, self.height-1)\nValid values for weight are in [this.minWeight, this.maxWeight]", py::arg("x"), py::arg("y"), py::arg("weight"));

    //PathPlanning
    py_wm.def("getPath", &WeightMap::getPath, "Returns the cheapest path between two points.", py::arg("src_x"), py::arg("src_y"), py::arg("dst_x"), py::arg("dst_y"));
    py_wm.def("pathToXVal", &WeightMap::pathToXVal, "Returns the cheapest path to the given x-value", py::arg("src_x"), py::arg("src_y"), py::arg("dst_x"));

    //AddBoarder
    py_wm.def_property_readonly_static("border_top", [](py::object /* self */){return BoarderPlace::TOP.get_value();});
    py_wm.def_property_readonly_static("border_bottom", [](py::object /* self */){return BoarderPlace::BOTTOM.get_value();});
    py_wm.def_property_readonly_static("border_right", [](py::object /* self */){return BoarderPlace::RIGHT.get_value();});
    py_wm.def_property_readonly_static("border_left", [](py::object /* self */){return BoarderPlace::LEFT.get_value();});
    py_wm.def("add_border", [](WeightMap& self, mapsize_t boarder_width, weight_t boarder_weight, int place){return self.addBoarder(boarder_width, boarder_weight, BoarderPlace(place));});

    py_wm.def("addObstical", &WeightMap::addObstical);

    py_wm.def("isValidPoint", [](WeightMap& self, py::int_ x_py, py::int_ y_py){
        PyObject* x_py_ptr = x_py.ptr();
        PyObject* y_py_ptr = y_py.ptr();

        int overflow = 0;
        const long x = PyLong_AsLongAndOverflow(x_py_ptr, &overflow);
        if(x == -1){
            //The user either passed a negitive value or a value out of bounds for a long
            //In either case, it is not a valid x-coordinate and we can return false
            return false;
        }

        const long y = PyLong_AsLongAndOverflow(y_py_ptr, &overflow);
        if(y == -1){
            //The user either passed a negitive value or a value out of bounds for a long
            //In either case, it is not a valid x-coordinate and we can return false
            return false;
        }

        //If the x-value is negitive or cannot fit inside a int32, we are guarenteed out of bounds
        if (x < 0 || x > (long)std::numeric_limits<int32_t>::max()){
            return false;
        }

        //If the y-value is negitive or cannot fit inside a int32, we are guarenteed out of bounds
        if (y < 0 || y > (long)std::numeric_limits<int32_t>::max()){
            return false;
        }

        return self.isValidPoint(x,y);
        }, "A method for checking the validity of coordinate points with respect to a WeightMap. Returns true if the point is valid, false otherwise." , py::arg("x"), py::arg("y"));

    //Python Dunder Methods
    py_wm.def("__repr__",
        [](const WeightMap &self) {
            std::string str(255, '\0');
            snprintf((char*)str.c_str(), str.size() * sizeof(str[0]), "<PyWeightMap.PyWeightMap at '%p'>", &self);

            return str;
        }
    );
    py_wm.def("__str__",
        [](const WeightMap &self) {
            return self.to_string(false);
        }
    );
    py_wm.doc() = "A dense 2D occupancy grid. Valid x coordinates are [0,self.width-1]. Valid y coordinates are [0, self.height-1]. ";
}