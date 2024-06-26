from setuptools import setup, find_packages
from glob import glob

package_name = 'cr2autoware'

setup(
    # package info
    name=package_name,
    version='0.1.0',
    zip_safe=True,
    author='Cyber-Physical Systems Group, Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    description='Interface between CommonRoad and Autoware.Universe',
    license='BSD',

    # Source
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/param', glob('param/*.param.yaml')),
    ],

    # Requirements
    python_requires='>=3.10',
    install_requires=[
    	'setuptools==69.0.2',
    	'commonroad-io==2024.1',
    	'commonroad-drivability-checker==2024.1',
    	'commonroad-vehicle-models==3.0.2',
    	'commonroad-route-planner==2024.2.0',
    	'pyproj>=3.4.1',
    	'pyyaml==6.0',
    	'utm>=0.7.0'
    	],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr2autoware_node = cr2autoware.cr2autoware:main',
        ],
    },

    # Additional Information
    classifiers=[
        "Programming Language :: Python :: 3.10",
        "Operating System :: POSIX :: Linux",
    ],
    # include *yaml configs in packaging
    include_package_data=True,
    package_data={'': ['*.yaml']}
)
