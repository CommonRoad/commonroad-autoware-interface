from setuptools import setup
from glob import glob

package_name = 'cr2autoware'

setup(
    # package info
    name=package_name,
    version='0.0.1',
    zip_safe=True,
    author='Cyber-Physical Systems Group, Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    description='Interface between CommonRoad and Autoware.Universe',
    license='TODO: License declaration',

    # Source
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/param', glob('param/*.param.yaml')),
    ],

    # Requirements
    python_requires='>=3.7',
    install_requires=[
    	'setuptools',
    	'commonroad-io==2022.3',
    	'commonroad-drivability-checker==2022.2.1',
    	'commonroad-vehicle-models>=3.0.0',
    	'commonroad-route-planner==2022.3',
    	'pyproj',
    	'pyyaml',
    	'utm'
    	],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr2autoware_node = cr2autoware.cr2autoware:main',
        ],
    },

    # Additional Information
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Operating System :: POSIX :: Linux",
    ],
)

