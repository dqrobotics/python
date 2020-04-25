#!/bin/bash

if [ "$TRAVIS_OS_NAME" = "osx" ]
then

    echo "[DQRobotics] Building for macOS"
    python3 setup.py bdist_wheel
    
else

    echo "[DQRobotics] Building for Ubuntu..."
    python3 --version
    python3 -m pip install setuptools wheel setuptools-git twine
    python3 setup.py bdist_wheel
    echo "[DQRobotics] Ubuntu build ended."
    
    echo "[DQRobotics] Installing..."
    python3 -m pip install dist/*.whl
    echo "[DQRobotics] Installed."
    
    echo "[DQRobotics] Testing..."
    python3 -c "from dqrobotics import *"
    python3 -m pip install --user scipy quadprog
    echo "[DQRobotics] Testing DQ..."
    python3 tests/DQ_test.py
    echo "[DQRobotics] Testing DQ_Kinematics..."
    python3 tests/DQ_Kinematics_test.py
    echo "[DQRobotics] Testing cpp_issues..."
    python3 tests/cpp_issues.py
    echo "[DQRobotics] Tested."
    
fi
