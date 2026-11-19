from glob import glob
from setuptools import find_packages, setup

setup(
    name='thesis_multi_robot', version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/thesis_multi_robot']),
        ('share/thesis_multi_robot', ['package.xml']),
        ('share/thesis_multi_robot/launch', glob('launch/*.launch.py')),
        ('share/thesis_multi_robot/worlds', glob('worlds/*.sdf')),
        ('share/thesis_multi_robot/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML', 'trimesh==4.4.9', 'pycollada==0.9.3'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Luis', maintainer_email='a20216480@pucp.edu.pe',
    description='Nominal simulation cell for four collaborative robot profiles.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'profile_supervisor = thesis_multi_robot.supervisor:main',
        'demo_obstacle = thesis_multi_robot.obstacle:main',
        'multi_robot_gui = thesis_multi_robot.multi_robot_gui:main',
    ]},
)
