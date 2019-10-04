#!/bin/bash
echo "Starting deployment at..."
pwd
ls
if [ "$TRAVIS_BRANCH" = "master" -a "$TRAVIS_PULL_REQUEST" = "false" ]
  then
    echo "Entering dist..."
    cd dist
    ls
    echo "Renaming wheel..."
    rename 's/linux/manylinux1/' *
    echo "Leaving dist..."
    cd ..
    ls
    echo "Uploading to twine..."
    python3 -m twine upload dist/*
fi
