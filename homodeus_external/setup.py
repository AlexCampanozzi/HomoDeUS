#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=["base_control","base_navigation","head_control","face_detection","keyword_detector"],
    package_dir={'': ''}
)
setup(**d)
