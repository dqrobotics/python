#!/bin/bash

echo "Building for 16.04 ..."
python3 setup.py bdist_wheel
sudo rm -r build
cd ..
python3 -m pip install --user ./python
python3 -c "from dqrobotics import *"
cd python
echo "16.04 build ended."
echo "Building for 18.04 ..."
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt-get update -q
sudo apt-get install python3.6 python3.6-dev -y
curl https://bootstrap.pypa.io/get-pip.py | sudo -H python3.6
python3.6 -m pip install --user setuptools wheel setuptools-git twine
python3.6 setup.py bdist_wheel
sudo rm -r build
cd ..
python3.6 -m pip install --user ./python
python3.6 -c "from dqrobotics import *"
echo "18.04 build ended."
echo "Testing..."
cd python
cd tests
python3.6 -m pip install --user scipy
echo "Testing DQ..."
python3.6 DQ_test.py
echo "Testing DQ_Kinematics..."
python3.6 DQ_Kinematics_test.py
echo "Testing ended."
