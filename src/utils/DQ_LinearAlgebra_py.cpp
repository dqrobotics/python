#include "../dqrobotics_module.h"

void init_DQ_LinearAlgebrapy(py::module& m)
{
    /*****************************************************
     *  DQ_LinearAlgebra
     * **************************************************/
    //#include<dqrobotics/utils/DQ_LinearAlgebra.h>
    py::module linearalgebra_py = m.def_submodule("DQ_LinearAlgebra","A submodule of utils");
    linearalgebra_py.def("pinv", (MatrixXd (*) (const MatrixXd&))&DQ_robotics::pinv, "Retrieves the pseudo-inverse of the input matrix");
}
