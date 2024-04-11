from setuptools import find_packages, setup

PACKAGE_NAME = 'aruco_follower'
DATA_FILES = [
    ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
    ('share/' + PACKAGE_NAME + '/launch', ['launch/robot_launch.py']),
    ('share/' + PACKAGE_NAME + '/worlds', [
        'worlds/mavic_playground.wbt',
        'worlds/aruco-marker-ID=228.png'
    ]),
    ('share/' + PACKAGE_NAME + '/resource', [
        'resource/aruco_follower.urdf',
        'resource/playground_supervisor.urdf'
    ]),
    ('share/' + PACKAGE_NAME, ['package.xml'])
]

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=DATA_FILES,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dsima',
    maintainer_email='simanokda.etu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_mavic_driver = aruco_follower.simple_mavic_driver:main'
            'playground_supervisor = aruco_follower.playground_supervisor:main'
        ],
    },
)
