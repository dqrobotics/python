#!/bin/bash

if [ "$TRAVIS_OS_NAME" = "osx" ]
then

    echo "Building for macOS"
    python3 setup.py bdist_wheel
    
else

    echo "Building for 16.04 (Xenial)..."
    python3 -m pip install pip
    python3 -m pip install --upgrade pip
    python3 setup.py bdist_wheel
    sudo rm -r build
    echo "16.04 build ended."

    echo "Building for 18.04 (Bionic)..."
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt-get update -q
    sudo apt-get install python3.6 python3.6-dev -y
    python3.6 -m pip install --user pip
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
    python3.6 -m pip install --user scipy quadprog
    echo "Testing DQ..."
    python3.6 DQ_test.py
    echo "Testing DQ_Kinematics..."
    python3.6 DQ_Kinematics_test.py
    echo "Testing cpp_issues..."
    python3.6 cpp_issues.py
    cd ..
    echo "Testing ended."

    echo "Building for 20.04 (Focal) ..."
    sudo apt-get install python3.8 -y
    sudo apt-get install python3.8-distutils -y
    sudo apt-get install python3.8-dev -y
    python3.8 -m pip install --user pip
    python3.8 -m pip install --user setuptools wheel setuptools-git
    python3.8 -m pip install --upgrade pip setuptools wheel
    sudo rm -r build
    python3.8 setup.py bdist_wheel
    echo "20.04 build ended."

fi
