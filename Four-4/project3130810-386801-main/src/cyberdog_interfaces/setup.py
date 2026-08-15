from setuptools import setup

package_name = 'cyberdog_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='l2826427290@foxmail.com',
    maintainer_email='l2826427290@foxmail.com',
    description='ROS 2 Python package for Cyberdog API communication interfaces and LCM message protocol structure.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = cyberdog_interfaces.test:main'
        ],
    },
)
