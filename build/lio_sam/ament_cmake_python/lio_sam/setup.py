from setuptools import find_packages
from setuptools import setup

setup(
    name='lio_sam',
    version='1.0.0',
    packages=find_packages(
        include=('lio_sam', 'lio_sam.*')),
)
