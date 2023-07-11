from setuptools import setup, find_packages

# Automatic version
from datetime import datetime
version_st = datetime.today().strftime('%y.%m.%d.%H%M%S')

# https://packaging.python.org/en/latest/guides/making-a-pypi-friendly-readme/
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='notros2',
    long_description=long_description,
    long_description_content_type='text/markdown',
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
