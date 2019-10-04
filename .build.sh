#!/bin/bash
echo "Installing twine..."
python3 -m pip install --user twine
echo "Compiling wheel..."
python3 setup.py bdist_wheel
echo "Deleting build folder..."
rm -rf build
echo "Install dqrobotics package..."
cd ..
python3 -m pip install --user ./python
cd python
echo "Try importing dqrobotics package to check for linking errors..."
python3 -c "from dqrobotics import *"
