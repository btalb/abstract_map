#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'abstract_map_lib', 'abstract_map_ros', 'abstract_map_visualiser'
    ],
    #scripts=['bin/myscript'],
    package_dir={'': 'src'})

setup(**d)
