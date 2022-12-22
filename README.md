# dqrobotics-python  [![Python package](https://github.com/dqrobotics/python/actions/workflows/python_package.yml/badge.svg)](https://github.com/dqrobotics/python/actions/workflows/python_package.yml) [![PyPI version](https://badge.fury.io/py/dqrobotics.svg)](https://badge.fury.io/py/dqrobotics) 
The DQ Robotics library in Python

Refer to the [docs](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html)


## Dev Stuff

With the initial settings described below for each system, the `actions_runner` will pick up the correct version of Python and run the CI accordingly. 

### Setting up the self-hosted CI enviroment for Apple silicon

1. Install `brew`
2. Install the Python versions currently supported by DQRobotics e.g. 
```
brew install python@3.XX
```
4. Add an alias for each Python version in your `.zshrc`, e.g. `alias python3.XX='/opt/homebrew/bin/python3.XX'`


### Setting up the self-hosted CI enviroment for Ubuntu arm64

0. Make sure you have the basic compilation enviroment
```
sudo apt update && sudo apt upgrade -y
sudo apt install git g++ cmake
```

1. Add the deadsnakes PPA
```
sudo add-apt-repository ppa:deadsnakes/ppa
```
2. Run 
```
sudo apt update
```
3. Install, for each Python version, the following 
```
sudo apt install python3.XX-dev python3.XX-venv python3.XX-distutils
```
