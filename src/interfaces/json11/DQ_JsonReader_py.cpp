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

#include "../../dqrobotics_module.h"

void init_DQ_JsonReader_py(py::module& m)
{
    py::class_<DQ_JsonReader> jsonreader_py(m,"DQ_JsonReader");
    jsonreader_py.def(py::init<>());

    jsonreader_py.def_static("get_from_json",&DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>,"Gets a DQ_KinematicsDH instance from a .json file");
    //This might be relevant in the future https://github.com/pybind/pybind11/issues/199
}
