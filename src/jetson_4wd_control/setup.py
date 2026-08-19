from setuptools import find_packages, setup

package_name = 'jetson_4wd_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Flash Sat Systems',
    maintainer_email='flash@example.com',
    description='Command velocity bridge for 4WD rover',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = jetson_4wd_control.cmd_vel_bridge:main',
        ],
    },
)
