from setuptools import setup
from glob import glob

package_name = 'cr2autoware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/param', glob('param/*.param.yaml')),
    ],
    install_requires=[
    	'setuptools',
    	'commonroad-io>=2022.3',
    	'commonroad-drivability-checker>=2021.4',
    	'commonroad-vehicle-models>=2.0.0',
    	'commonroad-route-planner>=2022.1',
    	'pyproj',
    	'pyyaml',
    	'utm'
    	],
    zip_safe=True,
    maintainer='drivingsim',
    maintainer_email='drivingsim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr2autoware_node = cr2autoware.cr2autoware:main',
        ],
    },
)
