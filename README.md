# dqrobotics-python  [![Python package](https://github.com/dqrobotics/python/actions/workflows/python_package.yml/badge.svg)](https://github.com/dqrobotics/python/actions/workflows/python_package.yml) [![PyPI version](https://badge.fury.io/py/dqrobotics.svg)](https://badge.fury.io/py/dqrobotics) 
The DQ Robotics library in Python

Refer to the [docs](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html)


## Dev Stuff

### Setting up the self-hosted enviroment for Apple silicon

1. Install `brew`
2. Install the Python versions currently supported by DQRobotics e.g. `brew install python@3.XX`
3. Add an alias for each Python version in your `.zshrc`, e.g. `alias python3.XX='/opt/homebrew/bin/python3.XX'`

With these settings, the `actions_runner` will pick up the correct version of Python and run the CI accordingly. 
