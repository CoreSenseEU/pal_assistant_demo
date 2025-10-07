#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup
from glob import glob

NAME = "skill_ask_human_for_help"

setup(
    name=NAME,
    version="1.0.0",
    license="Apache-2.0",
    description="Ask human for help",
    author="Sara Cooper",
    author_email="sara.cooper@pal-robotics.com",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + NAME, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['res/' + NAME]),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/pal_system_module',
         ['module/' + NAME]),
        ('share/' + NAME + '/module', ['module/' + NAME + '_module.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest']
    },
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'start_skill = ' + NAME + '.start_skill:main'
        ],
    },
)
