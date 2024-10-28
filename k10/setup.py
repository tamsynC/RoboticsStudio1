import os
from glob import glob
from setuptools import find_packages, setup
from ament_index_python.packages import get_package_share_directory

package_name = 'k10'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(
                'share', 
                package_name, 
                'launch'
                ),
            glob('launch/*.launch.py')
        ),
        (os.path.join(
                'share',
                package_name,
                'config'
                ),
            glob('config/*.yaml')
        ),
        (os.path.join(
                'share',
                package_name,
                'map'
                ),
            glob('map/*')
        ),
        (os.path.join(
                'share',
                package_name,
                'world'
                ),
            glob('world/*.world')
        ),
        (os.path.join(
                'share',
                package_name,
                'rviz'
                ),
            glob('rviz/*.rviz')
        ),
        (os.path.join(
                'share',
                'nav2_bringup',
                'launch'
                ),
            glob('launch/*.launch.py')
        )
    ],
    install_requires=['setuptools', 'nav2'],
    zip_safe=True,
    maintainer='joseph',
    maintainer_email='joseph.a.tarbath@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'k10 = k10.HQ:main',
        ],
    },
)
