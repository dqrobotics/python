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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include "../dqrobotics_module.h"

/**
 * @brief Binds `DQ_Geometry`, which provides geometric operations for points,
 * lines, planes, and line segments, to the Python module @p m.
 */
void init_DQ_Geometry_py(py::module& m)
{
    /*****************************************************
     *  DQ_Geometry
     * **************************************************/
    //#include<dqrobotics/utils/DQ_Geometry.h>
    py::class_<DQ_Geometry> geometry_py(
            m,
            "DQ_Geometry",
            "Provides geometric operations for points, lines, planes, and line segments.");
    geometry_py.def_static("point_to_point_squared_distance",
                           &DQ_Geometry::point_to_point_squared_distance,
                           py::arg("point1"),
                           py::arg("point2"),
                           "Computes the squared Euclidean distance between two points represented as pure quaternions.");
    geometry_py.def_static("point_to_line_squared_distance",
                           &DQ_Geometry::point_to_line_squared_distance,
                           py::arg("point"),
                           py::arg("line"),
                           "Computes the squared Euclidean distance between a point and a line.");
    geometry_py.def_static("point_to_plane_distance",
                           &DQ_Geometry::point_to_plane_distance,
                           py::arg("point"),
                           py::arg("plane"),
                           "Computes the signed distance from a point to a plane.");
    geometry_py.def_static("line_to_line_squared_distance",
                           &DQ_Geometry::line_to_line_squared_distance,
                           py::arg("line1"),
                           py::arg("line2"),
                           "Computes the squared Euclidean distance between two lines.");
    geometry_py.def_static("line_to_line_angle",
                           &DQ_Geometry::line_to_line_angle,
                           py::arg("line1"),
                           py::arg("line2"),
                           "Computes the angle between two lines in radians.");
    geometry_py.def_static("point_projected_in_line",
                           &DQ_Geometry::point_projected_in_line,
                           py::arg("point"),
                           py::arg("line"),
                           "Projects a point onto a line and returns the projected point as a pure quaternion.");
    geometry_py.def_static("closest_points_between_lines",
                           &DQ_Geometry::closest_points_between_lines,
                           py::arg("line1"),
                           py::arg("line2"),
                           "Computes and returns the closest point on each of two lines.");
    geometry_py.def_static("closest_points_between_line_segments",
                           &DQ_Geometry::closest_points_between_line_segments,
                           py::arg("line_1"),
                           py::arg("line_1_point_1"),
                           py::arg("line_1_point_2"),
                           py::arg("line_2"),
                           py::arg("line_2_point_1"),
                           py::arg("line_2_point_2"),
                           "Computes and returns the closest point on each of two valid line segments.");
    geometry_py.def_static("line_segment_to_line_segment_squared_distance",
                           &DQ_Geometry::line_segment_to_line_segment_squared_distance,
                           py::arg("line_1"),
                           py::arg("line_1_point_1"),
                           py::arg("line_1_point_2"),
                           py::arg("line_2"),
                           py::arg("line_2_point_1"),
                           py::arg("line_2_point_2"),
                           "Computes the squared Euclidean distance between two valid line segments.");

    geometry_py.def_static("is_line_segment",
                           &DQ_Geometry::is_line_segment,
                           py::arg("line"),
                           py::arg("line_point_1"),
                           py::arg("line_point_2"),
                           py::arg("threshold") = DQ_threshold,
                           "Checks whether a line and two endpoints define a valid line segment within the given threshold.");
    //Overload with the default threshold
    geometry_py.def_static("is_line_segment",
                           [](const DQ& line, const DQ& line_point_1, const DQ& line_point_2)
    {
        return DQ_Geometry::is_line_segment(line,line_point_1,line_point_2);
    },
    py::arg("line"),
    py::arg("line_point_1"),
    py::arg("line_point_2"),
    "Checks whether a line and two endpoints define a valid line segment using the library default threshold.");
}
