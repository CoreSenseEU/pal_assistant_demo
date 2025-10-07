#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup
from glob import glob

NAME = "skill_navigate_to_zone_sim"

setup(
    name=NAME,
    version="0.1.0",
    license="Apache-2.0",
    description="Simulated skill to navigate to a symbolic location (eg. kitchen)",
    author="todo",
    author_email="todo@todo.todo",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + NAME, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['res/' + NAME]),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
    ],
    tests_require=['pytest'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'start_skill = ' + NAME + '.start_skill:main'
        ],
    },
)
