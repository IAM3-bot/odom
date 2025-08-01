from setuptools import setup

package_name = 'encoder_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinho',
    maintainer_email='jinho@todo.todo',
    description='Encoder-based odometry publisher for ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = encoder_odom.odom_node:main',
        ],
    },
)
