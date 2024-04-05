from setuptools import find_packages, setup

package_name = 'aurco_follower'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/mavic_playground.wbt',
    'worlds/aruco-marker-ID=228.png'
]))
data_files.append(('share/' + package_name + '/resource', ['resource/aurco_follower.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dsima',
    maintainer_email='simanokda.etu@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_driver = aurco_follower.simple_mavic_driver:main',
        ],
    },
)