#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['vosk_asr'],
    package_dir={'': 'src'},
    install_requires=['vosk', 'sounddevice']) # vosk needs manual install

setup(**setup_args)
