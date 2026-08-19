#!/usr/bin/env python3

from setuptools import setup


package_name = "trajectory_generator"


setup(
    name=package_name,
    version="2.0.0",

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
            ],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=False,

    maintainer="flash",
    maintainer_email="flash@todo.todo",

    description=(
        "Dynamic mission loader, GPS-to-local converter "
        "and trajectory generator for the DYX 4WD rover."
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "trajectory_generator_node = "
                "trajectory_generator."
                "trajectory_generator_node:main"
            ),
        ],
    },
)