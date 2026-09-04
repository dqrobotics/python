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
 * @brief Binds `DQ_LinearAlgebra`, which provides linear-algebra utilities such
 * as matrix pseudoinversion, to the Python module @p m.
 */
void init_DQ_LinearAlgebra_py(py::module& m)
{
    /*****************************************************
     *  DQ_LinearAlgebra
     * **************************************************/
    //#include<dqrobotics/utils/DQ_LinearAlgebra.h>
    py::module linearalgebra_py = m.def_submodule("_DQ_LinearAlgebra","Linear-algebra utilities.");
    linearalgebra_py.def("pinv",
                         (MatrixXd (*) (const MatrixXd&))&DQ_robotics::pinv,
                         py::arg("matrix"),
                         "Computes the Moore-Penrose pseudoinverse of the input matrix.");
}
