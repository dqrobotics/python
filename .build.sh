#!/bin/bash
python3 -m pip install --user twine
python3 setup.py bdist_wheel
cd ..
python3 -m pip install --user ./python
cd python
python3 -c "from dqrobotics import *"
