from setuptools import find_packages, setup

package_name = "robotros_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotROS team",
    maintainer_email="todo@example.invalid",
    description="map_manager (so far: web map image, map meta, robot pose, snapshot; ADR 0005).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["map_manager = robotros_map.map_manager:main"]},
)
