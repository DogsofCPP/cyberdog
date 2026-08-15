from setuptools import setup

package_name = 'cyberdog_demo'

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
    description='ROS 2 Python demo package for Cyberdog examples and integration tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'adjust_node = cyberdog_demo.adjust_node:main',
            'test_demo = cyberdog_demo.test_demo:main',
            'test_demo4 = cyberdog_demo.test_demo4:main',
            'master = cyberdog_demo.master:main',
            'seek_001_node = cyberdog_demo.seek_001_node:main',
            'fisheyes_adjust_node = cyberdog_demo.fisheyes_adjust_node:main',
            'ground_node = cyberdog_demo.ground_node:main',
            'test3 = cyberdog_demo.test3:main',
            'fisheyes_ground_node = cyberdog_demo.fisheyes_ground_node:main',
        ],
    },
)
