#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'], 
    packages=['particle_filter_question'],
    package_dir={'': 'python'},
    scripts=[ 'python/monte_carlo_localization_v2.py',]
)

setup(**d)
