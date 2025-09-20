from setuptools import find_packages, setup

package_name = 'turtlesim_ext'

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
    maintainer='Bartlomiej Kulecki',
    maintainer_email='b.kulecki@gmail.com',
    description='A turtlesim extension package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_tracker = turtlesim_ext.pose_tracker:main',
            'turtle_behavior = turtlesim_ext.turtle_behavior:main'
        ],
    },
)
