from glob import glob

from setuptools import find_packages, setup

package_name = 'koreabot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vcs',
    maintainer_email='vcs@todo.todo',
    description='KoreaBot line follower',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'line_follow = koreabot.line_follow:main',
            'color_params = koreabot.color_params:main',
            'robot_params = koreabot.robot_params:main',
        ],
    },
)
