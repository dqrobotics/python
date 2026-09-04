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

#include "../../dqrobotics_module.h"

/**
 * @brief Binds `DQ_JsonReader`, which reads supported robot-description JSON
 * files and constructs serial manipulator objects from them, to the Python
 * module @p m.
 */
void init_DQ_JsonReader_py(py::module& m)
{
    py::class_<DQ_JsonReader> jsonreader_py(
            m,
            "DQ_JsonReader",
            "Reads supported robot-description JSON files and constructs serial manipulator objects from them.");
    jsonreader_py.def(py::init<>(), "Constructs a JSON reader instance.");

    jsonreader_py.def_static(
            "get_serial_manipulator_dh_from_json",
            &DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>,
            py::arg("file"),
            "Reads a JSON file describing a DQ_SerialManipulatorDH, converts angle fields according to the file's angle mode, initializes the common serial-manipulator properties, and returns the resulting model.");
    jsonreader_py.def_static(
            "get_serial_manipulator_denso_from_json",
            &DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>,
            py::arg("file"),
            "Reads a JSON file describing a DQ_SerialManipulatorDenso, converts angle fields according to the file's angle mode, initializes the common serial-manipulator properties, and returns the resulting model.");
    //This might be relevant in the future https://github.com/pybind/pybind11/issues/199
}
