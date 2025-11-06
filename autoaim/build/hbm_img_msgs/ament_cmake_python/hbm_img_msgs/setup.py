from setuptools import find_packages
from setuptools import setup

setup(
    name='hbm_img_msgs',
    version='2.0.0',
    packages=find_packages(
        include=('hbm_img_msgs', 'hbm_img_msgs.*')),
)
