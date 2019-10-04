#!/bin/bash

if [ "$TRAVIS_BRANCH" = "master" -a "$TRAVIS_PULL_REQUEST" = "false" ]
  then
    cd python
    cd dist
    rename 's/linux/manylinux1/' *
    cd ..
    python3 -m twine upload dist/*
fi
