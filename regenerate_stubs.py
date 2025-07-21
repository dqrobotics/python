"""
Fix the stubs created by pybind11-stubgen

The default version of pybind11 stubgen will generate something like

.
└── dqrobotics
    ├── __init__.pyi
    └── _dqrobotics
        ├── __init__.pyi
        ├── _interfaces
        │   ├── __init__.pyi
        │   └── _json11.pyi
        ├── _robot_control.pyi
        ├── _robot_modeling.pyi
        ├── _robots.pyi
        ├── _solvers.pyi
        └── _utils
            ├── _DQ_LinearAlgebra.pyi
            ├── _DQ_Math.pyi
            └── __init__.pyi

which is not compatible with dqrobotics' structure. It's not clear to me why that's the case.
This script will adjust it to something like below, which make the stubs quite useful.

.
├── README.md
├── __init__.pyi
├── interfaces
│   ├── __init__.pyi
│   └── json11
│       └── __init__.py
├── robot_control
│   └── __init__.py
├── robot_modeling
│   └── __init__.py
├── robots
│   └── __init__.py
├── solvers
│   └── __init__.py
└── utils
    ├── DQ_LinearAlgebra
    │   └── __init__.py
    ├── DQ_Math
    │   └── __init__.py
    └── __init__.pyi

Author: Murilo M. Marinho
"""

import os

def main():
    cwd = os.path.join(os.getcwd(),"stubs_temp")
    for root, dirs, files in os.walk(cwd, topdown=False):
        for name in files:
            if name.startswith('__'):
                continue
            elif name.startswith('_') and name.endswith('.pyi'):
                os.makedirs(os.path.join(root, name[1:-4]), exist_ok=True)
                os.rename(os.path.join(root, name), os.path.join(root, name[1:-4], "__init__.py"))
        for name in dirs:
            if name.startswith('_'):
                os.rename(os.path.join(root, name), os.path.join(root, name[1:]))

if __name__ == "__main__":
    main()