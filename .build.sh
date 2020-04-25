#!/bin/bash

if [ "$TRAVIS_OS_NAME" = "osx" ]
then

    echo "[DQRobotics] Building wheel for macOS"
    python3 setup.py bdist_wheel
    
else

    echo "[DQRobotics] Building wheel Ubuntu..."
    python3 --version
    python3 -m pip install setuptools wheel setuptools-git twine
    python3 setup.py bdist_wheel
    rm -rf build
    echo "[DQRobotics] Ubuntu wheel build ended."
    
    echo "[DQRobotics] Installing..."
    cd ..
    python3 -m pip install ./python
    cd python
    echo "[DQRobotics] Installed."
    
    echo "[DQRobotics] Testing..."
    cd tests
    python3 -c "from dqrobotics import *"
    python3 -m pip install scipy quadprog
    echo "[DQRobotics] Testing DQ..."
    python3 DQ_test.py
    echo "[DQRobotics] Testing DQ_Kinematics..."
    python3 DQ_Kinematics_test.py
    echo "[DQRobotics] Testing cpp_issues..."
    python3 cpp_issues.py
    cd ..
    echo "[DQRobotics] Tested."
    
fi
