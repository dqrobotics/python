#!/bin/bash

if [ "$TRAVIS_OS_NAME" = "osx" ]
then

    echo "Building for macOS"
    python3 setup.py bdist_wheel
    
else

    echo "Building for Ubuntu"
    python3 -m pip install --user pip
    python3 -m pip install --user setuptools wheel setuptools-git twine
    python3 setup.py bdist_wheel
    echo "Ubuntu build ended."
    
fi
