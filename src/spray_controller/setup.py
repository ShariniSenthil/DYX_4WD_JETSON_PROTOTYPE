from setuptools import find_packages, setup

package_name = "spray_controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="flash",
    maintainer_email="flash@todo.todo",
    description=(
        "Production PX4 AUX5 spray servo controller "
        "for the DYX marking rover"
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "spray_controller_node = "
                "spray_controller.spray_controller_node:main"
            ),
        ],
    },
)