import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


# Automatic name branching
def format_branch_name(name):
    if name == "master":
        # The master branch is considered an Alpha release
        return "a"
    elif name == "release":
        return ""
    else:
        return name


# read the contents of your README file
with open("README.md", "r") as fh:
    long_description = fh.read()


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        # Without this, the results of some calculations will differ from the MATLAB version and cause the tests to
        # fail.
        if platform.machine() == "aarch64":
            cmake_args += ['-DCMAKE_CXX_FLAGS="-ffp-contract=off"']
            # https://stackoverflow.com/questions/64036879/differing-floating-point-calculation-results-between-x86-64-and-armv8-2-a

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            cmake_args += ['-DCMAKE_TOOLCHAIN_FILE=C:/Tools/vcpkg/scripts/buildsystems/vcpkg.cmake']
            if sys.maxsize > 2 ** 32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)


setup(
    name='dqrobotics',
    version_config={
        "template": "{tag}.{branch}{ccount}",
        "dev_template": "{tag}.{branch}{ccount}",
        "dirty_template": "{tag}.{branch}{ccount}",
        "branch_formatter": format_branch_name,
    },
    setup_requires=['setuptools-git-versioning'],
    author='Murilo Marques Marinho',
    author_email='murilo@g.ecc.u-tokyo.ac.jp',
    description='DQRobotics Python',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url="https://github.com/dqrobotics/python",
    ext_modules=[CMakeExtension('dqrobotics._dqrobotics')],
    cmdclass=dict(build_ext=CMakeBuild),
    install_requires=[
        'numpy',
    ],
    zip_safe=False,
    packages=['dqrobotics',
              'dqrobotics.robots',
              'dqrobotics.robot_modeling',
              'dqrobotics.utils',
              'dqrobotics.utils.DQ_Math',
              'dqrobotics.utils.DQ_LinearAlgebra',
              'dqrobotics.interfaces',
              'dqrobotics.interfaces.vrep',
              'dqrobotics.interfaces.vrep.robots',
              'dqrobotics.interfaces.json11',
              'dqrobotics.robot_control',
              'dqrobotics.solvers'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: C++",
        "Development Status :: 5 - Production/Stable",
        "Operating System :: POSIX :: Linux",
        "License :: OSI Approved :: GNU Lesser General Public License v3 or later (LGPLv3+)",
    ],
)
