#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "WeightMap.hpp"
#include <cstdio>
#include <string>
#include <limits>
#include <stdexcept>

namespace py = pybind11;

mapsize_t mapsizeTFromPyInt(py::int_ &py_int)
{
    PyObject *const py_long = py_int.ptr();
    if (!PyLong_CheckExact(py_long))
    {
        throw std::invalid_argument("Argument was not a PyLong!");
    }

    const long value = PyLong_AsLong(py_long);
    if (PyErr_Occurred())
    {
        throw py::error_already_set();
    }

    if (value < 0)
    {
        throw std::domain_error("Value cannot be negitive");
    }

    if (value > (long)std::numeric_limits<uint16_t>::max())
    {
        throw std::overflow_error("Value greater than unsigned int max value and cannot be converted to unsigned int");
    }

    return value;
}

bool isValidPointShimFn(WeightMap &self, py::int_ x_py, py::int_ y_py)
{
    PyObject *x_py_ptr = x_py.ptr();
    PyObject *y_py_ptr = y_py.ptr();

    int overflow = 0;
    const long x = PyLong_AsLongAndOverflow(x_py_ptr, &overflow);
    if (x == -1)
    {
        // The user either passed a negitive value or a value out of bounds for a long
        // In either case, it is not a valid x-coordinate and we can return false
        return false;
    }

    const long y = PyLong_AsLongAndOverflow(y_py_ptr, &overflow);
    if (y == -1)
    {
        // The user either passed a negitive value or a value out of bounds for a long
        // In either case, it is not a valid x-coordinate and we can return false
        return false;
    }

    // If the x-value cannot fit inside a int32, we are guarenteed out of bounds
    if (x > (long)std::numeric_limits<int32_t>::max())
    {
        return false;
    }

    // If the y-value cannot fit inside a int32, we are guarenteed out of bounds
    if (y > (long)std::numeric_limits<int32_t>::max())
    {
        return false;
    }

    return self.isValidPoint(
        static_cast<int32_t>(x),
        static_cast<int32_t>(y));
}

constexpr const char *PyWeightMapDocString =
    R"mydelimiter(A Module containing occupancy grids and associated path finding methods.)mydelimiter";

constexpr const char *AddBorderDocString =
    R"myDelimiter(Adds a border to the edges of a weightmap.

:param border_width: The width of the border measuered from the edge of the map.
:type border_width: 16 bit unsigned int
:param border_weight: The weight the border will be set to.
:type border_weight: 16 bit unsigned int
:param border_place: A series of BorderPlace flags bitwise ORed together. 
:type border_place: An BorderPlace enum instance
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, the weight is an invalid weight or zero, or if the combination of border place enums passed is invalid.
:return: None
)myDelimiter";

constexpr const char *GetWeightDocString =
    R"myDelim(Returns the weight at the provided location

:param x: The x coordinate
:type x: 16 bit unsigned int
:param y: The y coordinate
:type y: 16 bit unsigned int
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, or the x-y coordinate point is outsize the map.
:return: The weight at that location
:rtype: int

)myDelim";

constexpr const char *SetWeightDocString =
    R"myDelim(Sets the weight at the provided location

:param x: The x coordinate
:type x: 16 bit unsigned int
:param y: The y coordinate
:type y: 16 bit unsigned int
:param weight: The weight to set that location to
:type weight: 16 bit unsigned int
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, the weight is out of bounds, or the x-y coordinate point is outsize the map.
:return: None

)myDelim";

constexpr const char *GetPathDocString =
    R"myDelim(Returns the cheapest path from the source point to the destination point

:param src_x: The x value of the source cordinate
:type src_x: 16 bit unsigned int
:param src_y: The y value of the source cordinate
:type src_y: 16 bit unsigned int
:param dst_x: The x value of the destination cordinate
:type dst_x: 16 bit unsigned int
:param dst_y: The y value of the destination cordinate
:type dst_y: 16 bit unsigned int
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, or a x-y coordinate point is outsize the map.
:return: A list of tuples with each tuple corresponding to an x-y point 
:rtype: list[tuple[int,int]]

)myDelim";

constexpr const char *PathToXDocString =
    R"myDelim(Returns the cheapest path from source point to the given x-value

:param src_x: The x value of the source cordinate
:type src_x: 16 bit unsigned int
:param src_y: The y value of the source cordinate
:type src_y: 16 bit unsigned int
:param dst_x: The x value of the destination cordinate
:type dst_x: 16 bit unsigned int
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, or a x-y coordinate point is outsize the map.
:return: A list of tuples with each tuple corresponding to an x-y point 
:rtype: list[tuple[int,int]]

)myDelim";

constexpr const char *AddObsticalDocString =
    R"myDelim(Adds a circular obsticle to the WeightMap centered at the point (x,y) with the given radius and weight. If gradient is specified, the weight in the circle decreases with respect to distance to WeightMap.min_weight

:param x: x component of the destination point
:type x: 16 bit unsigned int
:param y: y component of the destination point
:type y: 16 bit unsigned int
:param radius: The radius of the obstical
:type radius: 16 bit unsigned int
:param weight: The weight assigned to the obstical
:type weight: 16 bit unsigned int
:param gradient: If gradient is specified, the weight in the circle decreases with respect to distance from weight parameter to WeightMap.min_weight
:type gradient: bool
:raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
:raises ValueError: If any parameter passed is negitive, or a x-y coordinate point is outsize the map, or the weight is an invalid weight.
:return: None

)myDelim";

constexpr const char *IsValidPointDocString =
    R"myDelim(Checks the validity of a coordinate point. Returns true if the point is valid, false otherwise

:param x: x component of the point
:type x: int
:param y: y component of the point
:type y: int
:return: False if the point is out of bounds or if representing the point in C++ unsigned short would result in overflow. Returns true otherwise.
:rtype: bool

)myDelim";

constexpr const char *CompressPathDocString =
    R"myDelim(Compresses the path to the bare minimum required to represent the path. Eg, the path [(0,0), (1,1), (2,2)] is converted to the path [(0,0), (2,2)]

:param path: A list of tuples corresponding to x-y pairs of points
:type path: list[tuple[int,int]]
:raises ValueError: If any parameter passed is negitive or if any parameter passed cannot be converted to the propper C++ type resulting in an overflow.
:return: A compressed path where each point left in the path is a turning point in the path.
:rtype: list[tuple[int,int]]

)myDelim";

constexpr const char *IsPathContinuousDocString =
    R"myDelim(Tests if the provided path is continuous. Continuity is defined as having each point being within 1.4 units from each of its neighbors. This is used primairly for internal unit testing.

:param path: A list of tuples corresponding to x-y pairs of points
:type path: list[tuple[int,int]]
:raises ValueError: If any parameter passed is negitive or if any parameter passed cannot be converted to the propper C++ type resulting in an overflow.
:return: True if the path is continuous, false otherwise.
:rtype: bool

)myDelim";

constexpr const char *SmoothPathDocString =
    R"myDelim(Iteratively smooths a path to reduce number of points and decrease needed turns. 

:param path: A list of tuples corresponding to x-y pairs of points
:type path: list[tuple[int,int]]
:param smoothing_val: Controls the path smoothing. 1.0f is no smoothing, 0.0f turns the path into a straight line between the two end points
:type smoothing_val: float
:raises ValueError: If any parameter passed is negitive or if any parameter passed cannot be converted to the propper C++ type resulting in an overflow.
:return: A smoothed path
:rtype: list[tuple[int,int]]

)myDelim";

constexpr const char *fromBytesDocstring =
    R"myDelim(Unpacks a WeightMap from bytes.

:param bts: A byte string created by calling __bytes__ on a valid WeightMap instance
:type bts: bytes
:return: A weight map that is a copy of the original WeightMap
:rtype: WeightMap

)myDelim";

constexpr const char* WeightMapDocString = 
R"myDelim(A dense 2D occupancy grid for path planning. 
The map is devided into cells with a weight in each cell.
The weights range from [WeightMap.min_weight] to [WeightMap.max_weight] inclusively.
The weight dictates how challenging it is to move to that cell.
Weights can be manipulated on an individual cell level through get and set weight.
Weights can be maipulated in bulk through add_obstacle, and add_border methods.
A path can be generated by calling a pathing method like get_path.
The resulting path is the least costly path between two points.

Supports device independent serialization through pickle, bytes(), and WeightMap.from_bytes.
)myDelim";


PYBIND11_MODULE(PyWeightMap, m)
{
    m.doc() = PyWeightMapDocString; // optional module docstring

    auto border_place = py::enum_<BorderPlace>(m, "BorderPlace", "Enum for WeightMap.add_border. Values can be bitwise ORed together")
                            .value("TOP", BorderPlace::TOP, "Flag for adding a border to the TOP")
                            .value("BOTTOM", BorderPlace::BOTTOM, "Flag for adding a border to the BOTTOM")
                            .value("RIGHT", BorderPlace::RIGHT, "Flag for adding a border to the RIGHT")
                            .value("LEFT", BorderPlace::LEFT, "Flag for adding a border to the LEFT");

    border_place.def("__or__", [](const BorderPlace &a, const BorderPlace &b)
                     { return a | b; });

    // Path Utility
    m.def(
        "compress_path", [](WeightMap::path_t path)
        { 
        WeightMap::compressPath(path); 
        return path; },
        CompressPathDocString,
        py::arg("path"));

    m.def("is_path_continuous", &WeightMap::is_path_continuous,
          IsPathContinuousDocString,
          py::arg("path"));

    m.def(
        "smooth_path", [](WeightMap::path_t path, float smoothing_val)
        { 
        WeightMap::smoothPath(path, smoothing_val); 
        return path; },
        SmoothPathDocString, py::arg("path"), py::arg("smoothing_val"));

    auto py_wm = py::class_<WeightMap>(m, "WeightMap");

    py_wm.def(py::init<const mapsize_t, const mapsize_t>(),
              "Initializes to the given size [width x height]",
              py::arg("width"), py::arg("height"));

    // Data
    py_wm.def_property_readonly("width", &WeightMap::getWidth, "Width in cells");
    py_wm.def_property_readonly("height", &WeightMap::getHeight, "Height in cells");
    py_wm.def_property_readonly("max_value", &WeightMap::getMaxWeightInMap, "Maximum weight in the map");

    // Static Data
    py_wm.def_property_readonly_static(
        "max_weight", [](py::object /* self */)
        { return WeightMap::getMaxWeight(); },
        "The maximum weight that a WeightMap can hold"); // Going through lambda is necessary if it does not take arguements
    py_wm.def_property_readonly_static(
        "min_weight", [](py::object /* self */)
        { return WeightMap::getMinWeight(); },
        "The minimum weight a WeightMap can hold");

    // Weight setter and getter
    py_wm.def(
        "get_weight", [](WeightMap &self, py::int_ x, py::int_ y)
        { return self.getWeight(mapsizeTFromPyInt(x), mapsizeTFromPyInt(y)); },
        GetWeightDocString,
        py::arg("x"), py::arg("y"));

    py_wm.def(
        "set_weight", [](WeightMap &self, py::int_ x, py::int_ y, py::int_ weight)
        { return self.setWeight(mapsizeTFromPyInt(x), mapsizeTFromPyInt(y), mapsizeTFromPyInt(weight)); },
        SetWeightDocString,
        py::arg("x"), py::arg("y"), py::arg("weight"));

    // PathPlanning
    py_wm.def(
        "get_path", [](WeightMap &self, py::int_ src_x, py::int_ src_y, py::int_ dst_x, py::int_ dst_y)
        { return self.getPath(mapsizeTFromPyInt(src_x), mapsizeTFromPyInt(src_y), mapsizeTFromPyInt(dst_x), mapsizeTFromPyInt(dst_y)); },
        GetPathDocString,
        py::arg("src_x"), py::arg("src_y"), py::arg("dst_x"), py::arg("dst_y"));

    py_wm.def(
        "path_to_x", [](WeightMap &self, py::int_ src_x, py::int_ src_y, py::int_ dst_x)
        { return self.pathToXVal(mapsizeTFromPyInt(src_x), mapsizeTFromPyInt(src_y), mapsizeTFromPyInt(dst_x)); },
        PathToXDocString,
        py::arg("src_x"), py::arg("src_y"), py::arg("dst_x"));

    py_wm.def(
        "add_border", [](WeightMap &self, py::int_ width, py::int_ weight, BorderPlace place)
        { return self.addBoarder(mapsizeTFromPyInt(width), mapsizeTFromPyInt(weight), place); },
        AddBorderDocString,
        py::arg("border_width"), py::arg("border_weight"), py::arg("border_place"));

    py_wm.def(
        "add_obstical", [](WeightMap &self, py::int_ x, py::int_ y, py::int_ radius, py::int_ weight, bool gradient)
        { return self.addObstical(mapsizeTFromPyInt(x), mapsizeTFromPyInt(y), mapsizeTFromPyInt(radius), mapsizeTFromPyInt(weight), gradient); },
        AddObsticalDocString,
        py::arg("x"), py::arg("y"), py::arg("radius"), py::arg("weight"), py::arg("gradient"));

    py_wm.def("is_valid_point", &isValidPointShimFn,
              IsValidPointDocString,
              py::arg("x"), py::arg("y"));

    // Python string representation methods
    py_wm.def("__repr__",
              [](const WeightMap &self)
              {
                  std::string str(255, '\0');
                  snprintf((char *)str.c_str(), str.size() * sizeof(str[0]), "<PyWeightMap.PyWeightMap at '%p'>", &self);

                  return str;
              });
    py_wm.def("__str__",
              [](const WeightMap &self)
              {
                  return self.to_string(false);
              });

    py_wm.def("__eq__", &WeightMap::operator==);

    py_wm.def("__hash__", &WeightMap::hash);

    py_wm.def("get_weights",
        [](const WeightMap& self){
        // Allocate and initialize The data
        weight_t* weights = self.getWeights();

        // Create a Python object that will free the allocated
        // memory when destroyed:
        py::capsule free_when_done(weights, [](void *f) {
            free(f);
        });

        return py::array_t<weight_t>(
            {self.getWidth(), self.getHeight()}, // shape
            {sizeof(weight_t) , sizeof(weight_t)* self.getWidth()}, // C-style contiguous strides for double
            weights, // the data pointer
            free_when_done // numpy array references this parent
            ); 
        }
    );

    // Serialization

    const auto WeightMapToBytes = [](const WeightMap &wm) { // __getstate__
        // Return bytes that fully encodes the state of the object
        auto bytes = wm.serialize();
        auto bytes_out = py::bytes(bytes.first, bytes.second);
        free((void *)bytes.first);
        return bytes_out;

    };

    const auto WeightMapFromPyBytes = [](py::bytes t) { // __setstate__
        PyObject *const bytes_obj = t.ptr();

        std::pair<const char *, const size_t> bytes(PyBytes_AsString(bytes_obj), PyBytes_Size(bytes_obj));
        if (bytes.first == nullptr)
        {
            throw py::error_already_set();
        }

        return WeightMap::deserialize(bytes);
    };

    py_wm.def(py::pickle(WeightMapToBytes, WeightMapFromPyBytes));

    py_wm.def("__bytes__", WeightMapToBytes);

    py_wm.def_static("from_bytes", WeightMapFromPyBytes, py::arg("bts"), fromBytesDocstring);

    //Doc Strings
    
    py_wm.doc() = WeightMapDocString;
}