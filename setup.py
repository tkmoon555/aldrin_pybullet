from setuptools import setup, find_packages
import os
from glob import glob

setup(
    name='aldrin_pybullet',
    version='0.1.0',
    license='proprietary',
    description='Module demo',

    author='tkmoon555',
    author_email='--@gmail.com',
    url='None.com',

    packages=find_packages(where='scripts'),
    package_dir={
        '': 'scripts',
    },
)
