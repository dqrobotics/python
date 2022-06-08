/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include "../dqrobotics_module.h"

void init_DQ_Geometry_py(py::module& m)
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
    geometry_py.def_static("point_projected_in_line",          &DQ_Geometry::point_projected_in_line,         "Returns the point projected in a line.");
    geometry_py.def_static("closest_points_between_lines",     &DQ_Geometry::closest_points_between_lines,    "Returns the closest points between lines");
    geometry_py.def_static("line_segment_to_line_segment_squared_distance", &DQ_Geometry::line_segment_to_line_segment_squared_distance, "Returns the squared distance between two line segments.");
}

