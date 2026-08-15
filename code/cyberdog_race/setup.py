from setuptools import setup, find_packages

setup(
    name='cyberdog_race',
    version='1.0.0',
    description='CyberDog Race Controller for Xiaomi Cup 2026',
    author='CyberDog Race Team',
    packages=find_packages(),
    python_requires='>=3.8',
    # Per migration guide §2: the gait config files must be packaged so
    # `pip install -e .` on the real CyberDog2 keeps them reachable.
    package_data={
        'cyberdog_race': ['config/*.toml'],
    },
    include_package_data=False,
    install_requires=[
        'lcm>=1.4.0',
        'toml>=0.10.0',
        'opencv-python-headless>=4.5.0',
        'numpy>=1.20.0',
        'transforms3d>=0.4.0',
    ],
    entry_points={
        'console_scripts': [
            'cyberdog-race=cyberdog_race.race_main:main',
        ],
    },
)
