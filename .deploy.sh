#!/bin/bash

if [ "$TRAVIS_BRANCH" = "master" -a "$TRAVIS_PULL_REQUEST" = "false" ]
then
    if [ "$TRAVIS_OS_NAME" = "osx" ]
    then

        python3 -m pip install twine
        python3 -m twine upload dist/*
        
    else

        cd dist
        rename 's/linux/manylinux1/' *
        cd ..
        python3 -m twine upload dist/*
    
    fi
fi
