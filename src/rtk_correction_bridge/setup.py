from setuptools import find_packages, setup

package_name = 'rtk_correction_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flash',
    maintainer_email='flash@todo.todo',
    description='NTRIP RTCM correction bridge from Emlid caster to PX4 using MAVROS',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntrip_to_px4_node = rtk_correction_bridge.ntrip_to_px4_node:main',
        ],
    },
)