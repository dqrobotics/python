#!/bin/bash

python3 setup.py bdist_wheel
sudo rm -r build
cd ..
python3 -m pip install --user ./python
python3 -c "from dqrobotics import *"
cd python
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
