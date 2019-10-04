#!/bin/bash
pwd
cd python
ls
if [ "$TRAVIS_BRANCH" = "master" -a "$TRAVIS_PULL_REQUEST" = "false" ]
  then
    cd dist
    ls
    rename 's/linux/manylinux1/' *
    cd ..
    ls
    python3 -m twine upload dist/*
fi
