#include "../dqrobotics_module.h"

void init_DQ_Geometrypy(py::module& m)
{
    /*****************************************************
     *  DQ_Geometry
     * **************************************************/
    //#include<dqrobotics/utils/DQ_Geometry.h>
    py::class_<DQ_Geometry> geometry_py(m, "DQ_Geometry");
    geometry_py.def_static("point_to_point_squared_distance",  &DQ_Geometry::point_to_point_squared_distance, "Returns the squared distance between two points");
    geometry_py.def_static("point_to_line_squared_distance",   &DQ_Geometry::point_to_line_squared_distance,  "Returns the squared distance between a point and a line");
    geometry_py.def_static("point_to_plane_distance",          &DQ_Geometry::point_to_plane_distance,         "Returns the distance between a point and a plane");
    geometry_py.def_static("line_to_line_squared_distance",    &DQ_Geometry::line_to_line_squared_distance,   "Returns the squared distance between two lines");
    geometry_py.def_static("line_to_line_angle",               &DQ_Geometry::line_to_line_angle,              "Returns the angle between two lines");
}

