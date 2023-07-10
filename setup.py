# https://wiki.debian.org/Python/LibraryStyleGuide

from setuptools import setup, find_packages
from datetime import datetime

version_st = datetime.today().strftime('%y.%m.%d.%H%M%S')

setup(
    name='notros2',
    version=version_st,
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Murilo M. Marinho',
    maintainer_email='murilomarinho@ieee.org',
    description='notROS2 utilities.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'notros2 = notros2.notros2_script:main'
        ],
    },
)
