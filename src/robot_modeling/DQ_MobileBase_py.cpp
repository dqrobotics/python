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
 * @brief Binds `DQ_MobileBase`, the abstract mobile-base interface that
 * specializes `DQ_Kinematics` for mobile robots with an additional rigid frame
 * displacement, to the Python module @p m.
 */
void init_DQ_MobileBase_py(py::module& m)
{
    py::class_<
            DQ_MobileBase,
            std::shared_ptr<DQ_MobileBase>,
            DQ_Kinematics> dqmobilebase_py(
                m,
                "DQ_MobileBase",
                "Abstract class that defines an interface for mobile bases. It specializes DQ_Kinematics for mobile robots whose pose is described by a low-dimensional configuration vector and an additional rigid displacement from the raw mobile-base pose to the actual base frame.");
    dqmobilebase_py.def(
        "set_frame_displacement",
        &DQ_MobileBase::set_frame_displacement,
        py::arg("pose"),
        "Sets the rigid displacement from the raw mobile-base pose to the base frame.");
    dqmobilebase_py.def(
        "frame_displacement",
        &DQ_MobileBase::frame_displacement,
        "Returns the rigid displacement from the raw mobile-base pose to the base frame.");
}
