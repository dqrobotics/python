#!/bin/bash
set -e
sed -i'' -e 's/-j2/-j8/g' setup.py
source venv/bin/activate
cd ..
python -m pip uninstall dqrobotics -y
python -m pip install ./python
cd python/tests
python DQ_test.py
python DQ_Kinematics_test.py
python cpp_issues.py
python python_issues.py
cd ..
sed -i'' -e 's/-j8/-j2/g' setup.py
