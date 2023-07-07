from setuptools import setup
from glob import glob
import os
package_name = 'zhw_jaka_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*start.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fdse-arm',
    maintainer_email='fdse-arm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_line_wrapper=zhw_jaka_wrapper.move_line_wrapper:main',
            'ag95_gripper_wrapper=zhw_jaka_wrapper.ag95_gripper_wrapper:main'
        ],
    },
)
