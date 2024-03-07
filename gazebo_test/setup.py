from setuptools import find_packages, setup

package_name = 'gazebo_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danila Simanok',
    maintainer_email='simanokda.etu@gmail.com',
    description='Just a simple Gazebo test',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_commander = gazebo_test.gazebo_commander:main'
        ],
    },
)
