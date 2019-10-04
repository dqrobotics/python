#!/bin/bash
python3 -m pip install --user twine
python3 setup.py bdist_wheel
sudo rm -r build
cd ..
python3 -m pip install --user ./python
python3 -c "from dqrobotics import *"
cd python
