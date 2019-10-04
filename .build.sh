#!/bin/bash
pwd
ls
python3 -m pip install --user twine
pwd
ls
python3 setup.py bdist_wheel
pwd
ls
python3 -m pip install --user ./python
pwd
ls
python3 -c "from dqrobotics import *"
