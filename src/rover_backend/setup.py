#!/usr/bin/env python3

from setuptools import setup


package_name = "rover_backend"


setup(
    name=package_name,
    version="2.0.0",

    # Install only the final backend package.
    packages=[
        package_name,
    ],

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                f"resource/{package_name}",
            ],
        ),
        (
            f"share/{package_name}",
            [
                "package.xml",
                "requirements.txt",
            ],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=False,

    maintainer="Flash Sat Systems",

    maintainer_email=(
        "support@flashsatsystems.com"
    ),

    description=(
        "Production backend and ROS bridge "
        "for the DYX 4WD marking rover."
    ),

    license="Proprietary",

    entry_points={
        "console_scripts": [
            (
                "rover_backend = "
                "rover_backend.main:main"
            ),
            (
                "rtk_worker = "
                "rover_backend.rtk_worker_bootstrap:main"
            ),
        ],
    },
)