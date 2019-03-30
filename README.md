# dqrobotics-python
The DQ Robotics library in Python

# Installation in Ubuntu 16.04 or Ubuntu 18.04 (Users)

`python -m pip install dqrobotics`

or

`python3 -m pip install dqrobotics`

# Wheel compilation in Ubuntu 16.04/Ubuntu 18.04 (Developers)

Please stick to the approach above (install using pip) if you are a dqrobotics user.

## Requirements
### Python2
`sudo apt-get install python-pip python-dev python-setuptools python-wheel`

If you are allowed to upload to the dqrobotics's PyPi repo, the following is also needed

`python -m pip install twine`

### Python3
`sudo apt-get install python3-pip python3-dev python3-setuptools python3-wheel`

If you are allowed to upload to the dqrobotics's PyPi repo, the following is also needed

`python3 -m pip install twine`

## Build Wheel
The following commands will build a wheel compatible with your system in the folder `dist`.

### Python 2
On the root folder
`python setup.py bdist_wheel`

### Python 3
On the root folder
`python3 setup.py bdist_wheel`

## Upload to PyPi
Before uploading to PyPi each `.whl` filename has to be changed.

Change the name `something-linux_x86_64.whl` to `something-manylinux1_x86_x64.whl`

Then upload to PyPi

`python -m twine upload dist/*`

with your PyPi credentials.
