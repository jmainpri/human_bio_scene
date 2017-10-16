#!/usr/bin/env python
# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg import python_setup

# fetch values from package.xml
setup_args = python_setup.generate_distutils_setup(
    packages=['human_bio_scene'],
    package_dir={'': 'src'})

setup(**setup_args)
