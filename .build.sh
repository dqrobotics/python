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
    if [ $? -eq 0 ]
    then
      echo "Successfully imported dqrobotics."
    else
      echo "Failed importing dqrobotics"
      exit 1
    fi
    
    python3 -m pip install scipy quadprog
    
    echo "[DQRobotics] Testing DQ..."
    python3 DQ_test.py
    if [ $? -eq 0 ]
    then
      echo "Successfully tested DQ_test.py"
    else
      echo "Failed testing DQ_test.py"
      exit 1
    fi
    
    echo "[DQRobotics] Testing DQ_Kinematics..."
    python3 DQ_Kinematics_test.py
    if [ $? -eq 0 ]
    then
      echo "Successfully tested DQ_Kinematics_test.py"
    else
      echo "Failed testing DQ_Kinematics_test.py"
      exit 1
    fi
    
    echo "[DQRobotics] Testing cpp_issues..."
    python3 cpp_issues.py
    if [ $? -eq 0 ]
    then
      echo "Successfully tested cpp_issues.py"
    else
      echo "Failed testing cpp_issues.py"
      exit 1
    fi
    
    echo "[DQRobotics] Testing python_issues..."
    python3 python_issues.py
    if [ $? -eq 0 ]
    then
      echo "Successfully tested python_issues.py"
    else
      echo "Failed testing python_issues.py"
      exit 1
    fi
    
    cd ..
    echo "[DQRobotics] Tested."
    
fi
